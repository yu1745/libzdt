#!/usr/bin/env python3
"""ZDT_X42S 假电机（Emm 固件），监听 SocketCAN 接口，用于无硬件的端到端测试。

只依赖标准库。协议规则取自手册 V1.0.3：
  - 扩展帧，ID = (Addr << 8) | Packet，Packet 从 0 起；
  - 数据场首字节为功能码 Code，长命令每个包都重复 Code，后接至多 7 个命令字节；
  - 校验码固定 0x6B，只出现在最后一包；
  - 返回 02 命令正确 / E2 参数错误 / EE 格式错误 / 9F 动作完成（电机主动返回）。

用法: fake_motor.py [--iface vcan0] [--addr 1] [--log]
"""
import argparse
import socket
import struct
import sys
import threading
import time

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME = struct.Struct("=IB3x8s")
CHECKSUM = 0x6B

# 应答模式（手册 5.8.6 的 Response 菜单）
RESP_NONE, RESP_RECEIVE, RESP_REACHED, RESP_BOTH, RESP_OTHER = range(5)

PULSES_PER_REV = 3200       # 手册 p.56: 3200 脉冲 = 1 圈
COUNTS_PER_REV = 65536      # Emm 固件位置: 0-65535 表示 0-360°
GAP_TIMEOUT_S = 0.2         # 收到首包后等待续包的上限

# (功能码, 辅助码或 None) -> Code 之后的字节数（含辅助码与末尾校验）。
# 数值 = 手册各命令发送帧总长 - 2（地址与功能码）。
LENGTHS = {
    (0x06, 0x45): 2,    # 5.2.1 触发编码器校准        01 06 45 6B
    (0x0A, 0x6D): 2,    # 5.2.3 当前位置角度清零      01 0A 6D 6B
    (0x0E, 0x52): 2,    # 5.2.4 解除保护              01 0E 52 6B
    (0xF3, 0xAB): 4,    # 5.3.2 使能                  01 F3 AB 01 00 6B
    (0xF6, None): 6,    # 5.3.7 速度模式(Emm)         01 F6 01 05 DC 0A 00 6B
    (0xFD, None): 11,   # 5.3.12 位置模式(Emm)        01 FD 01 0F A0 00 00 01 FA 00 00 00 6B
    (0xFE, 0x98): 3,    # 5.3.13 立即停止             01 FE 98 00 6B
    (0xFF, 0x66): 2,    # 5.3.14 触发同步动作         01 FF 66 6B
    (0x4C, 0xAE): 18,   # 5.4.6 修改回零参数          20 字节
    (0x48, 0xD1): 31,   # 5.8.6 修改驱动配置(Emm)     33 字节
    (0x24, None): 1,    # 5.5.4 读总线电压            01 24 6B
    (0x36, None): 1,    # 5.5.13 读实时位置           01 36 6B
}


class Motor:
    def __init__(self, addr, log):
        self.addr = addr
        self.log = log
        self.response = RESP_RECEIVE    # 手册默认: Receive
        self.target = 0                 # 脉冲
        self.pos = 0                    # 脉冲
        self.cached = None              # 同步标志缓存的目标位置
        self.partial = {}               # 来源地址 -> dict(code, body, next, t)
        self.sock = None
        self.lock = threading.Lock()

    # ---- 发送 ----
    def send_reply(self, code, body):
        """body: Code 之后的内容，不含校验字节。长应答按命令同样的规则拆包。"""
        data = bytes(body) + bytes([CHECKSUM])
        packet = 0
        off = 0
        while True:
            chunk = data[off:off + 7]
            payload = bytes([code]) + chunk
            can_id = (self.addr << 8) | packet | CAN_EFF_FLAG
            frame = CAN_FRAME.pack(can_id, len(payload), payload.ljust(8, b"\x00"))
            with self.lock:
                self.sock.send(frame)
            off += len(chunk)
            packet += 1
            if off >= len(data):
                break
        if self.log:
            print("  <- %02X %s %02X" % (code, bytes(body).hex(" ").upper(), CHECKSUM), flush=True)

    def status(self, code, value):
        self.send_reply(code, [value])

    def ok(self, code, motion=False):
        if self.response in (RESP_RECEIVE, RESP_BOTH, RESP_OTHER):
            self.status(code, 0x02)
        if motion and self.response in (RESP_REACHED, RESP_BOTH):
            threading.Timer(0.1, self.status, args=(code, 0x9F)).start()

    # ---- 命令处理 ----
    def handle(self, code, body):
        """body 含辅助码与末尾校验字节。"""
        aux = body[0] if body else None
        if (code, aux) in LENGTHS:
            key = (code, aux)
        elif (code, None) in LENGTHS:
            key = (code, None)
        else:
            self.status(code, 0xEE)
            return
        if len(body) != LENGTHS[key] or body[-1] != CHECKSUM:
            self.status(code, 0xEE)
            return
        args = body[:-1]

        if key in ((0x06, 0x45), (0x0E, 0x52)):
            self.ok(code)
        elif key == (0x0A, 0x6D):
            self.target = self.pos = 0
            self.ok(code)
        elif key == (0xF3, 0xAB):
            en, sync = args[1], args[2]
            if en > 1 or sync > 1:
                self.status(code, 0xE2)
            else:
                self.ok(code)
        elif key == (0xF6, None):
            direction, speed, sync = args[0], (args[1] << 8) | args[2], args[4]
            if direction > 1 or speed > 3000 or sync > 1:
                self.status(code, 0xE2)
            else:
                self.ok(code)
        elif key == (0xFD, None):
            direction = args[0]
            speed = (args[1] << 8) | args[2]
            pulses = struct.unpack(">I", bytes(args[4:8]))[0]
            mode, sync = args[8], args[9]
            if direction > 1 or speed > 3000 or mode > 2 or sync > 1:
                self.status(code, 0xE2)
                return
            delta = -pulses if direction else pulses        # CW 为正
            if mode == 0:
                new = self.target + delta       # 相对上一次目标
            elif mode == 1:
                new = delta                     # 绝对
            else:
                new = self.pos + delta          # 相对当前
            if sync == 1:
                self.cached = new
            else:
                self.target = self.pos = new
            self.ok(code, motion=(sync == 0))
        elif key == (0xFE, 0x98):
            if args[1] > 1:
                self.status(code, 0xE2)
            else:
                self.ok(code)
        elif key == (0xFF, 0x66):
            if self.cached is not None:
                self.target = self.pos = self.cached
                self.cached = None
            self.ok(code)
        elif key == (0x4C, 0xAE):
            store, mode, direction = args[1], args[2], args[3]
            if store > 1 or mode > 4 or direction > 1:
                self.status(code, 0xE2)
            else:
                self.ok(code)
        elif key == (0x48, 0xD1):
            # 手册 p.101 的第 N 字节 = args[N-3]（args[0] 为辅助码 D1）；第 23 字节为应答方式
            reply_mode = args[20]
            if reply_mode > 4:
                self.status(code, 0xE2)
            else:
                self.response = reply_mode
                if self.log:
                    print("  response mode -> %d" % reply_mode, flush=True)
                self.ok(code)
        elif key == (0x24, None):
            self.send_reply(code, struct.pack(">H", 24000))     # 24.000 V
        elif key == (0x36, None):
            counts = abs(self.pos) * COUNTS_PER_REV // PULSES_PER_REV
            self.send_reply(code, bytes([1 if self.pos < 0 else 0]) + struct.pack(">I", counts))

    # ---- 接收 ----
    def on_frame(self, can_id, data):
        if not (can_id & CAN_EFF_FLAG):
            return
        ident = can_id & CAN_EFF_MASK
        addr, packet = (ident >> 8) & 0xFF, ident & 0xFF
        if ident >> 16 or (addr != self.addr and addr != 0) or not data:
            return
        code, chunk = data[0], bytes(data[1:])
        st = self.partial.get(addr)
        if packet == 0:
            st = self.partial[addr] = {"code": code, "body": b"", "next": 0, "t": time.time()}
        elif st is None or packet != st["next"] or code != st["code"]:
            self.partial.pop(addr, None)
            return
        st["body"] += chunk
        st["next"] = packet + 1
        st["t"] = time.time()

        aux = st["body"][0] if st["body"] else None
        key = (code, aux) if (code, aux) in LENGTHS else (code, None)
        expect = LENGTHS.get(key)
        # 完成条件: 已知长度且收齐，或不满 8 字节的包（最后一包）
        if (expect is not None and len(st["body"]) >= expect) or len(data) < 8:
            self.partial.pop(addr, None)
            if self.log:
                print("-> %02X %s" % (code, st["body"].hex(" ").upper()), flush=True)
            self.handle(code, st["body"])

    def run(self, iface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.1)
        print("fake motor addr=%d on %s ready" % (self.addr, iface), flush=True)
        while True:
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except socket.timeout:
                for a, st in list(self.partial.items()):
                    if time.time() - st["t"] > GAP_TIMEOUT_S:
                        self.partial.pop(a, None)
                continue
            can_id, dlc, data = CAN_FRAME.unpack(raw)
            self.on_frame(can_id, data[:dlc])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--log", action="store_true")
    args = ap.parse_args()
    try:
        Motor(args.addr, args.log).run(args.iface)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
