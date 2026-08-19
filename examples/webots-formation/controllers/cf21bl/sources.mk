# sources.mk — the substrate-agnostic source/include list, shared by this
# directory's own Makefile (the real Webots build) and ../../ci-check/
# Makefile (a header-stub, compile-only build with no Webots install —
# see that file). Nothing in this file beyond C_SOURCES/INCLUDE is
# specific to the cf21bl substrate; a new substrate's Makefile can
# include it unchanged (see ../../README.md's "Porting to a different
# element").
#
# CF21BL_DIR anchors every path below at this directory: "." (the
# default) when included from here, or a relative path back to here when
# included from elsewhere, since the two consumers run with different
# working directories (Makefile runs from here; ci-check/Makefile runs
# from ../../ci-check/).

CF21BL_DIR ?= .

TAPESTRY_ROOT      = $(CF21BL_DIR)/../../../..
TAPESTRY_OS        = $(TAPESTRY_ROOT)/tapestry-os
TAPESTRY_SDK       = $(TAPESTRY_ROOT)/sdk
TAPESTRY_TRANSPORT = $(TAPESTRY_OS)/subsys/transport
COMMON             = $(CF21BL_DIR)/../common

C_SOURCES = $(CF21BL_DIR)/main.c \
            $(CF21BL_DIR)/substrate_webots.c \
            $(CF21BL_DIR)/pid_controller.c \
            $(CF21BL_DIR)/choreo_telemetry.c \
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
