#!/usr/bin/env bash
# 端到端测试：QEMU(esp32c3) 里的固件经 TWAI 与宿主机上的假电机通信。
#
# 前置条件:
#   1. 已 source ESP-IDF 的 export.sh（IDF >= 6.1）
#   2. 已安装带 TWAI 的 QEMU:  python $IDF_PATH/tools/idf_tools.py install qemu-riscv32
#   3. 宿主机存在虚拟 CAN 网卡（需要 root，重启后失效）:
#        sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
#
# 用法: tools/run_e2e.sh [超时秒数，默认 90]
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
app="$root/test_apps/e2e"
iface=${ZDT_CAN_IFACE:-vcan0}
limit=${1:-90}
work=$(mktemp -d)

: "${IDF_PATH:?请先 source \$IDF_PATH/export.sh}"
ip link show "$iface" >/dev/null 2>&1 || { echo "找不到 CAN 接口 $iface，见脚本头部说明" >&2; exit 2; }

if ! command -v qemu-system-riscv32 >/dev/null; then
    qemu_bin=$(ls -d "$HOME"/.espressif/tools/qemu-riscv32/*/qemu/bin 2>/dev/null | sort -V | tail -1 || true)
    [ -n "$qemu_bin" ] || { echo "未安装 qemu-riscv32，见脚本头部说明" >&2; exit 2; }
    export PATH="$qemu_bin:$PATH"
fi

motor_pid=""
qemu_pid=""
cleanup() {
    [ -n "$qemu_pid" ] && kill -TERM -- "-$qemu_pid" 2>/dev/null || true
    [ -n "$motor_pid" ] && kill -TERM "$motor_pid" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

python3 -I "$root/tools/fake_motor.py" --iface "$iface" --log >"$work/motor.log" 2>&1 &
motor_pid=$!
sleep 1

extra="-object can-bus,id=canbus0 -object can-host-socketcan,id=canhost0,if=$iface,canbus=canbus0"
extra="$extra -global driver=esp32c3.twai,property=canbus,value=canbus0"

cd "$app"
[ -f sdkconfig ] || idf.py set-target esp32c3 >"$work/set-target.log" 2>&1
setsid timeout -k 2 "$limit" idf.py qemu --qemu-extra-args "$extra" >"$work/qemu.log" 2>&1 &
qemu_pid=$!

for _ in $(seq 1 "$limit"); do
    sleep 1
    grep -a -q -E 'E2E (SUMMARY|FATAL)' "$work/qemu.log" && break
    kill -0 "$qemu_pid" 2>/dev/null || break
done

grep -a -E '^E2E ' "$work/qemu.log" || true
if grep -a -q -E '^E2E SUMMARY pass=[0-9]+ fail=0[[:space:]]*$' "$work/qemu.log"; then
    echo "e2e: OK  (日志: $work)"
    exit 0
fi
echo "e2e: FAILED  (日志: $work/qemu.log, $work/motor.log)" >&2
exit 1
