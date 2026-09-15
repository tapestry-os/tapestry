# sources.mk — source/include list for the warehouse AMR controller, shared
# by this directory's Makefile (the real Webots build) and ../../ci-check/
# (a header-stub, compile-only build with no Webots install).
#
# ROVER_DIR anchors every path below at this directory: "." when included
# from here, or a relative path back to here when included from elsewhere,
# since the two consumers run with different working directories.
#
# REUSE, NOT A FORK. controllers/common/ is taken from
# examples/webots-formation by relative path and compiled unmodified — the
# UDP transceiver, the Zephyr shim that lets gossip.c build outside Zephyr,
# the Webots stubs, and tracker.c's target-leash/repulsion/arena-clamp
# follower. That example's README calls that directory substrate-agnostic;
# this example is the test of that claim, since it is a ground vehicle and
# that one flies. Nothing in it was changed to accommodate this example
# except two additive changes, both no-ops for the drone build:
#   - udp_posix_set_link_blocked() (transceiver_udp_posix.h), unused there
#   - #ifndef guards around tracker.h's DEMO_* constants, overridden below

ROVER_DIR ?= .

TAPESTRY_ROOT      = $(ROVER_DIR)/../../../..
TAPESTRY_OS        = $(TAPESTRY_ROOT)/tapestry-os
TAPESTRY_SDK       = $(TAPESTRY_ROOT)/sdk
TAPESTRY_TRANSPORT = $(TAPESTRY_OS)/subsys/transport
COMMON             = $(TAPESTRY_ROOT)/examples/webots-formation/controllers/common

C_SOURCES = $(ROVER_DIR)/main.c \
            $(ROVER_DIR)/substrate_webots.c \
            $(ROVER_DIR)/spring_track.c \
            $(ROVER_DIR)/avoid.c \
            $(ROVER_DIR)/rf_occlusion.c \
            $(ROVER_DIR)/status_tx.c \
            $(ROVER_DIR)/scene.c \
            $(ROVER_DIR)/script_scene1.c \
            $(ROVER_DIR)/script_scene2.c \
            $(ROVER_DIR)/script_scene3.c \
            $(COMMON)/transceiver_udp_posix.c \
            $(COMMON)/tracker.c \
            $(TAPESTRY_OS)/subsys/csm/world_model.c \
            $(TAPESTRY_OS)/subsys/scr/scr.c \
            $(TAPESTRY_OS)/subsys/bse/bse.c \
            $(TAPESTRY_OS)/subsys/choreo/choreo.c \
            $(TAPESTRY_TRANSPORT)/gossip.c

INCLUDE += -I$(TAPESTRY_OS)/include \
           -I$(TAPESTRY_SDK)/include \
           -I$(TAPESTRY_TRANSPORT) \
           -I$(COMMON) \
           -I$(COMMON)/zephyr_shim

# Warehouse-scale overrides for tracker.h's #ifndef-guarded constants. The
# drone example's values are tuned for 0.1 m airframes in a 6 m room; these
# are 0.5 m ground vehicles in a 20x14 m warehouse, so every one of them is
# genuinely wrong at this scale rather than merely conservative:
#
#   MAX_SPEED   0.3 -> 0.7 m/s   the rate the COMMANDED setpoint advances.
#                                At 0.3 m/s a 12 m aisle transit takes 40 s
#                                and would blow the scripts' step timeouts.
#                                Stays below substrate_webots.c's 0.9 m/s
#                                motor ceiling so the setpoint, not the
#                                motor, is what limits travel.
#   MIN_SEP     0.5 -> 0.8 m     emergency-repulsion floor. The AMR body is
#                                0.5 m long, so 0.5 m centre-to-centre is
#                                inside the hull; 0.8 m leaves ~0.3 m clear.
#   ARENA_LIMIT 5.0 -> 13.0 m    per-axis clamp on the setpoint. 5.0 m would
#                                silently clamp scene 3's x = +/-7 m pick
#                                face, quietly collapsing the formation
#                                instead of failing loudly.
#   LEASH       0.75 -> 1.5 m    how far ahead of the body the setpoint may
#                                get. Scaled with the speed increase.
CFLAGS += -DDEMO_MAX_SPEED_MPS=0.7f \
          -DDEMO_MIN_SEP_M=0.8f \
          -DDEMO_ARENA_LIMIT_M=13.0f \
          -DDEMO_TARGET_LEASH_M=1.5f
