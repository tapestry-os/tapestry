# sources.mk — the cutebot substrate's source/include list, shared by this
# directory's own Makefile (the real Webots build) and
# ../../ci-check-cutebot/Makefile (a header-stub, compile-only build with
# no Webots install — see that file). Modeled on ../cf21bl/sources.mk —
# same split, same TAPESTRY_ROOT anchoring. No transport.c here: this
# substrate assigns element_id from controllerArgs (see main.c's header
# comment), not transport_negotiate_id(), so it never needs it.
#
# CUTEBOT_DIR anchors every path below at this directory: "." (the
# default) when included from here, or a relative path back to here when
# included from elsewhere.

CUTEBOT_DIR ?= .

TAPESTRY_ROOT      = $(CUTEBOT_DIR)/../../../..
TAPESTRY_OS        = $(TAPESTRY_ROOT)/tapestry-os
TAPESTRY_SDK       = $(TAPESTRY_ROOT)/sdk
TAPESTRY_TRANSPORT = $(TAPESTRY_OS)/subsys/transport
COMMON              = $(CUTEBOT_DIR)/../common

C_SOURCES = $(CUTEBOT_DIR)/main.c \
            $(CUTEBOT_DIR)/substrate_webots_cutebot.c \
            $(CUTEBOT_DIR)/tracker.c \
            $(COMMON)/transceiver_udp_posix.c \
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
