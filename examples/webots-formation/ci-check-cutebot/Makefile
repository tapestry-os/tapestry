# ci-check-cutebot/Makefile — compile-and-link check with NO Webots
# install, for the cutebot substrate. Sibling to ../ci-check/ (cf21bl) —
# see that file's comment for the full rationale; this is the same
# pattern applied to controllers/cutebot/ instead.
#
# Usage: cd ci-check-cutebot && make

CUTEBOT_DIR := ../controllers/cutebot
include $(CUTEBOT_DIR)/sources.mk

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -g
BUILD   := build

# See ../ci-check/Makefile's identical comment: the zephyr_shim's
# k_uptime_get_32() needs POSIX clock_gettime()/CLOCK_MONOTONIC, hidden
# under -std=c99's __STRICT_ANSI__ without this.
POSIX_DEFS := -D_POSIX_C_SOURCE=200809L

STUB_DIRS    := webots_stub ../controllers/common/webots_stub
ALL_SOURCES  := $(C_SOURCES) webots_stub/webots_stub.c
ALL_INCLUDE  := $(INCLUDE) $(addprefix -I,$(STUB_DIRS))
OBJECTS      := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(notdir $(ALL_SOURCES)))))
BINARY       := $(BUILD)/cutebot_ci_check

vpath %.c . $(sort $(dir $(ALL_SOURCES)))

.PHONY: all clean
all: $(BINARY)

$(BINARY): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) $(POSIX_DEFS) $(ALL_INCLUDE) -c -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
