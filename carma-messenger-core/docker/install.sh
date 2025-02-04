#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
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

# Get ubuntu distribution code name. All STOL APT debian packages are pushed to S3 bucket based on distribution codename.
. /etc/lsb-release

# add the STOL APT repository
echo "deb [trusted=yes] http://s3.amazonaws.com/stol-apt-repository ${DISTRIB_CODENAME} main" | sudo tee /etc/apt/sources.list.d/stol-apt-repository.list

sudo apt-get update

# install carma-j2735 library for encoding/decoding of messages
sudo apt-get install -y stol-j2735-1

# Build ros2
source /opt/ros/humble/setup.bash
cd ~/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/carma/install
