# ci-check/Makefile — compile-and-link check with NO Webots install, plus
# geometry tests that actually run.
#
# Targets, doing different jobs:
#
#   link-check       builds the whole rover controller — every Tapestry
#                    source from L3 to L7, this example's substrate, and
#                    the shared controllers/common/ files — against stub
#                    Webots headers, and LINKS it. Never run: it catches
#                    compile and signature breakage, nothing more. Same
#                    tradeoff, and same stub approach, as
#                    ../../webots-formation/ci-check/.
#
#   sup-check        compiles and links the read-only status readout in
#                    ../controllers/warehouse_supervisor/. Also never run.
#
#   cloud-link-check compiles and links ../controllers/cloud_bot/ and
#                    ../controllers/cloud_coordinator/ (scenes 2/3's
#                    centralized-coordination comparison fleet). Also
#                    never run.
#
#   geom-check       builds and RUNS rf_geom_check.c, which asserts that
#                    scene 2's script geometry and the RF occlusion model
#                    still agree (see that file for why that needs
#                    testing), and cloud_geom_check.c, which asserts the
#                    same for scenes 2/3's cloud fleet — the check that
#                    would have caught, before ever launching Webots, the
#                    cloud fleet's first placement physically wedging a
#                    robot against the deck's support leg (see
#                    cloud_geom_check.c's header comment for the full
#                    story).
#
# Usage: cd ci-check && make
#
# Neither compile target substitutes for building against the real Webots
# SDK (see ../controllers/rover/Makefile) before trusting a change to the
# device calls themselves.

ROVER_DIR := ../controllers/rover
include $(ROVER_DIR)/sources.mk

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -g
BUILD   := build

# k_uptime_get_32() in the shared zephyr_shim calls clock_gettime(
# CLOCK_MONOTONIC) — POSIX, not ISO C, and -std=c99 defines __STRICT_ANSI__
# under which glibc hides both it and struct timespec. Declaring the POSIX
# level keeps the strict-C99 conformance check on the Tapestry sources,
# which is the point of this build, rather than relaxing to -std=gnu99.
#
# _DARWIN_C_SOURCE is the macOS half of the same problem, and it is why
# ../../webots-formation/ci-check/ does not currently build on a Mac at all:
# Apple's libc hides INADDR_LOOPBACK (netinet/in.h) whenever _POSIX_C_SOURCE
# is defined without it, so transceiver_udp_posix.c fails to compile there
# while passing cleanly on Linux CI. Both are no-ops on Linux.
POSIX_DEFS := -D_POSIX_C_SOURCE=200809L -D_DARWIN_C_SOURCE

STUB_DIRS    := webots_stub $(COMMON)/webots_stub
ALL_SOURCES  := $(C_SOURCES) webots_stub/webots_stub.c
ALL_INCLUDE  := $(INCLUDE) -I$(ROVER_DIR) $(addprefix -I,$(STUB_DIRS))
OBJECTS      := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(notdir $(ALL_SOURCES)))))

SUP_DIR   := ../controllers/warehouse_supervisor
CLOUD_BOT_DIR  := ../controllers/cloud_bot
CLOUD_COORD_DIR := ../controllers/cloud_coordinator

LINK_BIN       := $(BUILD)/rover_ci_check
GEOM_BIN       := $(BUILD)/rf_geom_check
CLOUD_GEOM_BIN := $(BUILD)/cloud_geom_check
SUP_BIN        := $(BUILD)/supervisor_ci_check
CLOUD_BOT_BIN  := $(BUILD)/cloud_bot_ci_check
CLOUD_COORD_BIN := $(BUILD)/cloud_coordinator_ci_check

# Deliberately NOT a blanket "vpath %.c ... $(CLOUD_BOT_DIR) $(CLOUD_COORD_DIR)":
# rover/main.c, cloud_bot/main.c, and cloud_coordinator/main.c all share the
# bare name "main.c", and the generic pattern rule below resolves a bare
# "main.c" prerequisite (as rover's own $(OBJECTS) list does, via
# notdir-stripped "main.o") through whichever vpath directory is searched
# first — a blanket rule silently pointed rover's OWN build/main.o at
# cloud_bot's main.c instead once cloud_bot's directory was added, and the
# resulting link failure ("undefined reference to cloud_link_obstructed" in
# what should have been the pure-Tapestry rover binary) was the symptom.
# cloud_occlusion.c has no such collision (it exists in exactly one
# directory), so it gets its own filename-scoped vpath entry instead of
# riding along on a blanket one; cloud_bot's and cloud_coordinator's own
# main.c files are built by the fully-qualified-path explicit rules below,
# which need no vpath search at all.
vpath %.c . $(SUP_DIR) $(sort $(dir $(ALL_SOURCES)))
vpath cloud_occlusion.c $(CLOUD_BOT_DIR)

.PHONY: all link-check geom-check sup-check cloud-link-check clean
all: link-check sup-check cloud-link-check geom-check

link-check: $(LINK_BIN)

geom-check: $(GEOM_BIN) $(CLOUD_GEOM_BIN)
	@echo "--- rf_geom_check ---"
	@./$(GEOM_BIN)
	@echo "--- cloud_geom_check ---"
	@./$(CLOUD_GEOM_BIN)

$(LINK_BIN): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# The geometry check needs only the occlusion model, but that pulls in the
# transceiver for udp_posix_set_rx_filter().
$(GEOM_BIN): $(BUILD)/rf_geom_check.o $(BUILD)/rf_occlusion.o \
             $(BUILD)/transceiver_udp_posix.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Cross-checks the cloud fleet's independent occlusion duplicate
# (cloud_occlusion.c) against the Tapestry fleet's own (rf_occlusion.c) —
# needs both, plus the transceiver rf_occlusion.c links against.
$(CLOUD_GEOM_BIN): $(BUILD)/cloud_geom_check.o $(BUILD)/cloud_occlusion.o \
                   $(BUILD)/rf_occlusion.o $(BUILD)/transceiver_udp_posix.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

# cloud_geom_check.c lives here in ci-check/, not next to cloud_occlusion.h,
# so its quote-include needs an explicit -I (unlike cloud_occlusion.c
# itself, whose matching quote-include resolves against its own vpath-found
# directory with no extra flag needed).
$(BUILD)/cloud_geom_check.o: cloud_geom_check.c | $(BUILD)
	$(CC) $(CFLAGS) $(POSIX_DEFS) $(ALL_INCLUDE) -I$(CLOUD_BOT_DIR) -c -o $@ $<

sup-check: $(SUP_BIN)

$(SUP_BIN): $(BUILD)/warehouse_supervisor.o $(BUILD)/webots_stub.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

cloud-link-check: $(CLOUD_BOT_BIN) $(CLOUD_COORD_BIN)

# cloud_bot reuses rover's substrate_webots.c/status_tx.c and its own
# cloud_occlusion.c — see cloud_bot/main.c's header comment for why it
# links no Tapestry L3-L7 source at all.
$(CLOUD_BOT_BIN): $(BUILD)/cloud_bot_main.o $(BUILD)/cloud_occlusion.o \
                  $(BUILD)/substrate_webots.o $(BUILD)/status_tx.o \
                  $(BUILD)/webots_stub.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(BUILD)/cloud_bot_main.o: $(CLOUD_BOT_DIR)/main.c | $(BUILD)
	$(CC) $(CFLAGS) $(POSIX_DEFS) $(ALL_INCLUDE) -I$(CLOUD_BOT_DIR) -c -o $@ $<

# cloud_coordinator has no Webots device access at all beyond
# wb_robot_init()/wb_robot_step() (see its main.c) — just the shared
# status_tx.c and the robot.h stub every other target already needs.
$(CLOUD_COORD_BIN): $(BUILD)/cloud_coordinator_main.o $(BUILD)/status_tx.o \
                    $(BUILD)/webots_stub.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(BUILD)/cloud_coordinator_main.o: $(CLOUD_COORD_DIR)/main.c | $(BUILD)
	$(CC) $(CFLAGS) $(POSIX_DEFS) $(ALL_INCLUDE) -c -o $@ $<

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) $(POSIX_DEFS) $(ALL_INCLUDE) -c -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
