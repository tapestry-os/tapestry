# Standard Webots controller Makefile for the cloud-comparison robot.
#
# Deliberately no sources.mk / no Tapestry L3-L7 sources at all — that
# absence is the point (see main.c's header comment). Reuses exactly two
# files from ../rover/ by relative path, unmodified: substrate_webots.c
# (the differential-drive L1 HAL — pure motor/sensor plumbing, no
# coordination logic) and status_tx.c (the presentation-only telemetry
# feed shared by both fleets).

C_SOURCES = main.c \
            cloud_occlusion.c \
            ../rover/substrate_webots.c \
            ../rover/status_tx.c

INCLUDE += -I../../../../tapestry-os/include

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
