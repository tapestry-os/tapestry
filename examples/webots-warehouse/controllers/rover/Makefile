# Standard Webots controller Makefile — same pattern as
# ../../../webots-formation/controllers/cf21bl/Makefile, which carries the
# fuller explanation of why the Tapestry core sources are pulled in by
# relative path rather than built through Zephyr/west.
#
# Build:
#   export WEBOTS_HOME=/Applications/Webots.app
#   export WEBOTS_HOME_PATH=/Applications/Webots.app/Contents
#   make
#
# (On macOS those two differ by /Contents: Webots' Makefile system wants
# WEBOTS_HOME to be the bundle root for library/include paths but
# WEBOTS_HOME_PATH to be where resources/Makefile.include actually lives.
# A "webots/robot.h file not found" failure is almost always this.)

include sources.mk

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Webots' auto-dependency (.d) rules target $(BUILD_GOAL_DIR)/<obj>.o, which
# does not exist on a macOS fat-binary build — the real objects live in
# $(BUILD_GOAL_DIR)/{arm64,x86_64}/. Without these explicit rules, editing a
# .choreo.toml and regenerating its header silently fails to trigger a
# rebuild. Same fix, and same reason, as the cf21bl Makefile's.
SCRIPT_OBJS := script_scene1.o script_scene2.o script_scene3.o
SCRIPT_HDRS := scene1/choreo_script.h scene2/choreo_script.h scene3/choreo_script.h
$(addprefix $(BUILD_GOAL_DIR)/,$(SCRIPT_OBJS)) \
$(addprefix $(BUILD_GOAL_DIR)/x86_64/,$(SCRIPT_OBJS)) \
$(addprefix $(BUILD_GOAL_DIR)/arm64/,$(SCRIPT_OBJS)): $(SCRIPT_HDRS)
