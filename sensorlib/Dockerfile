FROM python:3.8.10

WORKDIR /home/CarlaCDASimAdapter/

COPY ./src /home/CarlaCDASimAdapter/
COPY requirements.txt /home/CarlaCDASimAdapter/
RUN pip install --upgrade pip
## Install Dependencies
RUN apt-get update
RUN apt-get install -y gfortran wget
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.tar.gz 
RUN tar -xf CARLA_0.9.10.tar.gz
ENV LOAD_CARLA_EGG=True
ENV CARLA_VERSION=0.9.10
ENV CARLA_EGG_DIR="/home/CarlaCDASimAdapter/CARLA_0.9.10/PythonAPI/carla"
RUN pip install -r requirements.txt
WORKDIR /home/CarlaCDASimAdapter/src/

CMD [ "python", "CarlaCDASimAdapter.py --carla-host 127.0.0.1" ]