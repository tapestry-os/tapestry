# ci-check/Makefile — compile-and-link check with NO Webots install.
#
# controllers/cf21bl/Makefile pulls in $(WEBOTS_HOME)/resources/Makefile.
# include, which needs an actual Webots install (~1 GB) for its headers,
# libcontroller, and build machinery. That's the right tool for actually
# running this example, but too heavy to ask of every CI run just to
# catch "does this still compile" — the same tradeoff ci.yml's hw-build
# job already makes for targets CI can't run either (ESP32, RA8D1, real
# Crazyflie hardware): compile (here, also link) it, never run it.
#
# This directory contains no real substrate code of its own — everything
# it builds is pulled in by relative path from ../controllers/cf21bl/ and
# ../controllers/common/, unmodified, so what's compiled here is exactly
# what the real Webots build compiles. webots_stub/ (plus
# ../controllers/common/webots_stub/, the substrate-agnostic half)
# declares just the ~10 wb_* functions cf21bl's main.c/substrate_webots.c
# actually call, matching the real Webots C API signatures, plus trivial
# definitions so this also links — catching signature mismatches a
# syntax-only check would miss. It is NOT the real Webots API surface and
# NOT a substitute for building against the real SDK (see
# ../README.md's Build section) before trusting a change that touches
# the Webots device calls themselves.
#
# Usage: cd ci-check && make

CF21BL_DIR := ../controllers/cf21bl
include $(CF21BL_DIR)/sources.mk

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -g
BUILD   := build

# k_uptime_get_32() in ../controllers/common/zephyr_shim/zephyr/kernel.h
# calls clock_gettime(CLOCK_MONOTONIC).  Those are POSIX, not ISO C, and
# -std=c99 defines __STRICT_ANSI__, under which glibc hides both them and
# struct timespec — so the shim stops compiling on Linux while the real
# Webots build (Webots' Makefile.include does not pass -std=) is fine.
# Declaring the POSIX level the shim actually needs keeps the strict-C99
# conformance check on the Tapestry sources, which is the point of this
# build, instead of relaxing it to -std=gnu99.
#
# It cannot live in the shim header: a feature-test macro must precede the
# first libc include, and gossip.c includes <string.h> before it reaches
# <zephyr/kernel.h>.  Kept out of CFLAGS so that overriding CFLAGS on the
# command line cannot silently drop it.
POSIX_DEFS := -D_POSIX_C_SOURCE=200809L

STUB_DIRS    := webots_stub ../controllers/common/webots_stub
ALL_SOURCES  := $(C_SOURCES) webots_stub/webots_stub.c
ALL_INCLUDE  := $(INCLUDE) $(addprefix -I,$(STUB_DIRS))
OBJECTS      := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(notdir $(ALL_SOURCES)))))
BINARY       := $(BUILD)/cf21bl_ci_check

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
