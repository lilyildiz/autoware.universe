# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(stanley)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
        SYSTEM
        ${EIGEN3_INCLUDE_DIRS}
)

# stanley_core
ament_auto_add_library(stanley_core SHARED
        src/stanley_core/stanley.cpp
        src/stanley_core/stanley_utils.cpp
        )

# stanley
ament_auto_add_library(stanley_lateral_controller SHARED
        src/stanley_lateral_controller.cpp
        )

target_link_libraries(stanley_lateral_controller
        stanley_core
        )

rclcpp_components_register_node(stanley_lateral_controller
        PLUGIN "autoware::stanley::StanleyLateralController"
        EXECUTABLE stanley_lateral_controller_exe
        )


# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
