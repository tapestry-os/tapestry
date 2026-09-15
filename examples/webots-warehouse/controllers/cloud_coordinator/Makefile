# Standard Webots controller Makefile for the central "cloud" coordinator.
#
# No Webots device access at all beyond wb_robot_init()/wb_robot_step() —
# this process doesn't drive anything or sense anything; it exists purely
# to run a clock and a UDP socket. Reuses ../rover/status_tx.c unmodified
# for its own heartbeat to the supervisor (see main.c).

C_SOURCES = main.c \
            ../rover/status_tx.c

INCLUDE +=

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
