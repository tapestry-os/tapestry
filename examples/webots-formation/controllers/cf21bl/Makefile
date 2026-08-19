# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Standard Webots controller Makefile (same pattern as
# projects/robots/bitcraze/crazyflie/controllers/crazyflie/Makefile) plus
# the include paths needed to pull the real Tapestry L3-L7 core sources in
# unmodified. See ../../README.md for the full picture.
#
# COMMON holds the substrate-agnostic pieces shared by every controller in
# this example (see ../common/ and ../../README.md's "Porting to a
# different element" section) — nothing in this Makefile beyond
# COMMON/C_SOURCES/INCLUDE is specific to the cf21bl substrate. That list
# lives in sources.mk, shared with ../../ci-check/Makefile (a header-stub,
# compile-only build with no Webots install, run from a directory that
# contains no real substrate code at all — see that file).

include sources.mk

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Webots' own auto-dependency (.d) rules target $(BUILD_GOAL_DIR)/main.o,
# which doesn't exist on a macOS fat-binary build -- the real objects are
# $(BUILD_GOAL_DIR)/{arm64,x86_64}/main.o, so a change to a header main.c
# includes (choreo_script.h) silently fails to trigger a rebuild. Declare
# the real dependency explicitly, for both fat and non-fat (NO_FAT_BINARY)
# object layouts; BUILD_GOAL_DIR is defined by the include above.
$(BUILD_GOAL_DIR)/main.o $(BUILD_GOAL_DIR)/x86_64/main.o $(BUILD_GOAL_DIR)/arm64/main.o: choreo_script.h
