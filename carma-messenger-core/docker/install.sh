#!/bin/bash

#  Copyright (C) 2018-2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

set -e

# Build ros2
source /opt/ros/humble/setup.bash

if [[ ! -z "$PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
fi

cd ~/

if [[ ! -z "$PACKAGES" ]]; then
    echo "Incrementally building following packages and those dependent on them: $PACKAGES"
    colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $PACKAGES
else
    echo "Building all CARMA Messenger components..."
    colcon build  --install-base /opt/carma/install --build-base build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo "Build of CARMA Messenger Components Complete"
