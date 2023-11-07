# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
FROM usdotfhwastoldev/carma-base:develop

LABEL Description="Dockerised CARMA-CARLA integration"

ARG CARMA_VERSION="develop"
ARG VERSION
ARG VCS_REF
ARG BUILD_DATE

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-carla-integration"
LABEL org.label-schema.description="XIL carma-carla-integration for evaluation and testing of the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-simulation/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

USER root
RUN apt-get update && apt-get install -y sudo

USER carma
WORKDIR /home/carma
COPY PythonAPI ./PythonAPI
COPY requirements.txt ./requirements.txt





RUN pip3 install -r /home/carma/requirements.txt
COPY / ./lib
COPY /tests ./tests

ENV LOAD_CARLA_EGG=True
ENV CARLA_VERSION=0.9.10
ENV CARLA_EGG_DIR=/home/carma/PythonAPI/carla/dist
ENV CARLA_HOST=127.0.0.1

CMD [ "python3", "/home/carma/tests/integration/simulated_sensor_open3d_lidar.py" ]
#CMD [ "python3", "/home/CarlaCDASimAdapter/lib/carla_cda_sim_adapter.py", "--carla-host", "${CARLA_HOST}", "--xmlrpc-server-host", "127.0.0.1",  "--sensor-config-file", "config/simulated_sensor_config.yaml", "--noise-model-config-file", "config/noise_model_config.yaml" ]
