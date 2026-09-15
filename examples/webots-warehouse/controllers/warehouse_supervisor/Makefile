# Webots controller Makefile for the read-only status readout.
#
# It compiles exactly one file plus the status struct definition it shares
# with the rover controller. It links nothing from Tapestry: this process is
# not an element and has no L3-L7 stack — see warehouse_supervisor.c.

INCLUDE += -I../rover

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
