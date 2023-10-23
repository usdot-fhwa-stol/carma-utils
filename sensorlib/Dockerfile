FROM ubuntu:focal
WORKDIR /home/CarlaCDASimAdapter/
COPY ./src/ /home/CarlaCDASimAdapter/src
COPY requirements.txt /home/CarlaCDASimAdapter/
COPY ./config/ /home/CarlaCDASimAdapter/config
## Install Dependencies
RUN apt-get update
RUN apt-get install -y gfortran wget python3 python3-pip libtiff5 libjpeg-dev libpng16-16
RUN wget https://github.com/usdot-fhwa-stol/carma-carla-integration/raw/develop/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
ENV LOAD_CARLA_EGG=True
ENV CARLA_VERSION=0.9.10
ENV CARLA_EGG_DIR="/home/CarlaCDASimAdapter/"
RUN pip3 install -r requirements.txt
WORKDIR /home/CarlaCDASimAdapter/
CMD [ "python3", "/home/CarlaCDASimAdapter/src/CarlaCDASimAdapter.py", "--carla-host", "127.0.0.1", "--xmlrpc-server-host", "127.0.0.1",  "--sensor-config-file", "../config/simulated_sensor_config.yaml", "--noise-model-config-file", "../config/noise_model_config.yaml" ]