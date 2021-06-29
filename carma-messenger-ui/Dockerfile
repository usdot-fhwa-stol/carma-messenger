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

# CARMA Web Interface Docker Configuration Script
#
# Performs all necessary tasks related to generation of a docker image set up
# to serve the CARMA web-based user interface via Apache and interact with the
# host's docker daemon as needed to start and stop the CARMA application itself
# 
# In order to enable the web-start functionality please run this image with
# -v /var/run/docker.sock:/var/run/docker.sock

FROM php:7.1-apache-jessie

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-messenger-ui"
LABEL org.label-schema.description="Web user interface for the CARMA Messenger"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-messenger"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --chown=www-data website /var/www/html
ADD docker/install.sh /
RUN /install.sh
