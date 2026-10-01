# Host build of the portable firmware core, for tests on a PC (Linux, macOS or WSL).
#   make test      unit tests
#   make harness   stand-in ESP32 used by relay/tests/test_c_client.py
CC ?= cc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined
CORE := ../../main
SRCS := $(CORE)/sha256.c $(CORE)/mavframe.c $(CORE)/tunnel.c $(CORE)/snapshot.c $(CORE)/mavpos.c $(CORE)/locator.c $(CORE)/alarm.c

all: test_core tunnel_harness

test_core: test_core.c $(SRCS) $(wildcard $(CORE)/*.h)
	$(CC) $(CFLAGS) -I$(CORE) -o $@ test_core.c $(SRCS) -lm

tunnel_harness: tunnel_harness.c $(SRCS) $(wildcard $(CORE)/*.h)
	$(CC) $(CFLAGS) -I$(CORE) -o $@ tunnel_harness.c $(SRCS) -lm

test: test_core
	./test_core

harness: tunnel_harness

clean:
	rm -f test_core tunnel_harness

.PHONY: all test harness clean
