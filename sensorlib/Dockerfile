FROM ubuntu:focal

## Install Dependencies
RUN apt-get update
RUN apt-get install -y gfortran wget python3 python3-pip libtiff5 libjpeg-dev libpng16-16
WORKDIR /home/CarlaCDASimAdapter/
RUN wget https://github.com/usdot-fhwa-stol/carma-carla-integration/raw/develop/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

ENV LOAD_CARLA_EGG=True
ENV CARLA_VERSION=0.9.10
ENV CARLA_EGG_DIR="/home/CarlaCDASimAdapter/"

WORKDIR /home/CarlaCDASimAdapter/
RUN pip3 install numpy PyYAML scipy dataclasses

COPY ./src/ /home/CarlaCDASimAdapter/src
COPY ./config/ /home/CarlaCDASimAdapter/config
