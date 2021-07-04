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

#
# GENERIC Build Script for CARMA Configuration Images
#
# Do not run this script itself, rather symlink this script into the individual
# configuration folders below and invoke it there to build the appropriate config
# image using docker build. Automatically acquires folder name and system version
# and passes neessary data into the docker build process.

USERNAME=usdotfhwastol
IMAGE=carma-messenger-config
cd "$(dirname "$0")"
DIR_NAME=${PWD##*/}
CONFIG_NAME=`echo $DIR_NAME | sed 's/_/-/g'`
COMPONENT_VERSION_STRING=$1

echo ""
echo "##### CARMA $CONFIG_NAME Configuration Docker Image Build Script #####"
echo ""

while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -d|--develop)
            USERNAME=usdotfhwastoldev
            COMPONENT_VERSION_STRING=noetic-develop
            shift
            ;;
    esac
done

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    TAG="$("../docker/get-system-version.sh")-$CONFIG_NAME"
else
    TAG="$COMPONENT_VERSION_STRING-$CONFIG_NAME"
fi

echo "Building docker image for CARMA Configuration version: $TAG"
echo "Final image name: $USERNAME/$IMAGE:$TAG"

docker build --no-cache -t $USERNAME/$IMAGE:$TAG \
    --build-arg VERSION="$TAG" \
    --build-arg VCS_REF=`git rev-parse --short HEAD` \
    --build-arg CONFIG_NAME="$IMAGE:$CONFIG_NAME" \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` .

echo ""
echo "##### CARMA $CONFIG_NAME Docker Image Build Done! #####"
