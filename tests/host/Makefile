# 主机单元测试：make -C tests/host test
SRC := $(wildcard ../../src/zdt_common_*.c) ../../src/zdt_x.c ../../src/zdt_emm.c ../../src/zdt_frame.c
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined
BUILD := build

test: $(BUILD)/test_ttl
	./$(BUILD)/test_ttl

$(BUILD)/test_ttl: test_ttl.c hw_cases.inc libzdt_host.h $(SRC) $(wildcard ../../include/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I../../include -I. -o $@ test_ttl.c $(SRC)

clean:
	rm -rf $(BUILD)

.PHONY: test clean
