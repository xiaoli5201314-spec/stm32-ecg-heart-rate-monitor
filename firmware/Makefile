# ---------------------------------------------------------------------------
# STM32 ECG acquisition node
#
#   make test      build and run the host unit-test suite (default target)
#   make demo      build and run the end-to-end acquisition demo
#   make dump      write the raw/filtered CSVs used by tools/verify_filters.py
#   make lib       build a static archive of the firmware sources
#   make arm       cross-compile for the STM32F103 target (needs arm-none-eabi-gcc)
#   make size      report the host object sizes per translation unit
#   make clean
#
# Verified with GNU Make 4.3 and gcc 11.4 on Ubuntu 22.04 (WSL).
# ---------------------------------------------------------------------------

CC      ?= gcc
AR      ?= ar
CROSS   ?= arm-none-eabi-

BUILD   := build
INC     := -Iinclude -Ihal

SRC      := $(wildcard src/*.c)
HALSTUB  := hal/hal_stub.c
TESTSRC  := $(wildcard test/*.c)
LIBSRC   := $(SRC) $(HALSTUB)

WARN := -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes \
        -Wpointer-arith -Wwrite-strings -Wundef -Wswitch-enum -Wredundant-decls

# -MMD -MP generates a .d file per object listing the headers it consumed, so a
# change in include/*.h rebuilds exactly the translation units that need it.
# Without this a struct layout change would leave stale objects behind.
DEPFLAGS := -MMD -MP

CFLAGS  ?= -std=c99 -O2 -g $(WARN) $(INC) $(DEPFLAGS)
LDFLAGS ?=
LDLIBS  ?= -lm

# -Werror is on by default so that `make test` also proves the sources are
# warning free; override with `make test WERROR=0`.
WERROR ?= 1
ifeq ($(WERROR),1)
CFLAGS += -Werror
endif

# The threaded ring-buffer stress test needs pthreads; it is built on POSIX
# hosts and skipped on Windows.
UNAME_S := $(shell uname -s 2>/dev/null)
ifneq (,$(findstring Linux,$(UNAME_S)))
CFLAGS  += -DECG_HAVE_PTHREAD
LDLIBS  += -pthread
endif

OBJ  := $(patsubst %.c,$(BUILD)/%.o,$(LIBSRC)) $(patsubst %.c,$(BUILD)/%.o,$(TESTSRC))
DEPS := $(OBJ:.o=.d)

-include $(DEPS)

.PHONY: all test demo dump lib arm size clean help

all: test

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/run_tests: $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(BUILD)/libecg.a: $(patsubst %.c,$(BUILD)/%.o,$(SRC))
	$(AR) rcs $@ $^

lib: $(BUILD)/libecg.a

test: $(BUILD)/run_tests
	@echo "----------------------------------------------------------------"
	@echo "running: $(BUILD)/run_tests"
	@echo "----------------------------------------------------------------"
	@./$(BUILD)/run_tests

demo: $(BUILD)/run_tests
	@./$(BUILD)/run_tests demo

dump: $(BUILD)/run_tests
	@./$(BUILD)/run_tests --dump $(BUILD)
	@echo "wrote $(BUILD)/raw_signal.csv and $(BUILD)/filtered_signal.csv"

size: $(OBJ)
	@size -t $(OBJ) 2>/dev/null || echo "size: not available on this host"

# --- cross build for the real target -------------------------------------
# Only the firmware sources are compiled (hal_stub.c is replaced by the board
# port on the target).  A minimal startup file is not part of this repository,
# so `make arm` stops after producing the objects: it is a syntax / warning
# check with the real toolchain, not a linkable image.
arm:
	@command -v $(CROSS)gcc >/dev/null 2>&1 || { \
		echo "arm: $(CROSS)gcc not found - install the GNU Arm Embedded toolchain"; \
		echo "arm: skipping (this target only checks syntax and warnings)"; \
		exit 0; }
	@set -e; for f in $(SRC); do \
		echo "  cc $$f"; \
		$(CROSS)gcc -std=c99 -O2 -mcpu=cortex-m3 -mthumb -Wall -Wextra \
			$(INC) -c $$f -o /dev/null; \
	done
	@echo "arm: all firmware sources compile clean for Cortex-M3"

clean:
	rm -rf $(BUILD)

help:
	@sed -n '2,12p' Makefile
