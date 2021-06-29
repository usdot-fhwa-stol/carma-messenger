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

# Script sets up the /opt/carma-messenger folder for the user
# Takes in one argument which is the path to the vehicle installation folder to use

GRP_ID=1000
sudo groupadd --gid $GRP_ID carma # create the carma messenger group if it does not already exist and add current user to it
USERNAME=$(whoami)
sudo usermod -a -G $GRP_ID $USERNAME

sudo mkdir -p /opt/carma-messenger/Desktop /opt/carma-messenger/Pictures 
sudo chgrp -R $GRP_ID /opt/carma-messenger/
sudo chmod 775 -R /opt/carma-messenger/
sudo chmod 775 /opt/carma-messenger/Desktop /opt/carma-messenger/Pictures


curl -o /opt/carma-messenger/Desktop/CARMA_Messenger.desktop -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-messenger/develop/install_scripts/Desktop/CARMA_Messenger.desktop 
chmod 775 /opt/carma-messenger/Desktop/CARMA_Messenger.desktop


curl -o /opt/carma-messenger/Pictures/CARMA_icon_color.png  https://raw.githubusercontent.com/usdot-fhwa-stol/carma-messenger/develop/install_scripts/Pictures/CARMA_icon_color.png 
chmod 775 /opt/carma-messenger/Pictures/CARMA_icon_color.png

cp /opt/carma-messenger/Desktop/CARMA_Messenger.desktop ~/Desktop/
chmod 775 ~/Desktop/CARMA_Messenger.desktop


