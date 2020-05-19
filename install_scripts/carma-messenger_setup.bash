#!/bin/bash

#  Copyright (C) 2018-2020 LEIDOS.
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

# Script sets up the /opt/carma-messenger folder for the user
# Takes in one argument which is the path to the vehicle installation folder to use

#if [ ! -d "$1" ]; then
#    echo "Please specify a path to the location of your vehicle installation folder. Such as carma-messenger-config/example_installation_folder/vehicle/"
#    exit -1
#fi

GRP_ID=1000
sudo groupadd --gid $GRP_ID carma # create the carma messenger group if it does not already exist and add current user to it
USERNAME=$(whoami)
sudo usermod -a -G $GRP_ID $USERNAME

#sudo mkdir -p /opt/carma-messenger/Desktop /opt/carma-messenger/Pictures 
#sudo chgrp -R $GRP_ID /opt/carma-messenger/
#sudo chmod 775 -R /opt/carma-messenger/
#sudo chmod 775 /opt/carma-messenger/Desktop /opt/carma-messenger/Pictures


curl -o ~/Desktop/CARMA Messenger.desktop -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-messenger/tree/feature/login_logout/install_scripts/Desktop/CARMA_Messenger.desktop
chmod 775 /opt/carma-messenger/Desktop/CARMA Messenger.desktop



curl -o ~/Pictures/CARMA_icon_color.png -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-messenger/tree/feature/login_logout/install_scripts/Pictures/CARMA_icon_color.png 
chmod 775 /opt/carma-messenger/Pictures/CARMA_icon_color.png

cd /opt/carma-messenger/Desktop/
ln -s ../Pictures/CARMA_icon_color.png  CARMA_icon_color.png

ln -s "$1" /opt/carma/vehicle
