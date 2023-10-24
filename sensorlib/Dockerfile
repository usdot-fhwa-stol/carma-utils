FROM python


# Install CARLA .egg file(s)
WORKDIR /tmp
#https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_${CARLA_VERSION}.tar.gz
#tar -xf CARLA_${CARLA_VERSION}.tar.gz
#COPY --from=0 /tmp/CARLA_${CARLA_VERSION} /usr/lib/carla/

# Copy source and configuration files
WORKDIR /usr/src/sensorlib
COPY . /usr/src/sensorlib


# Install Python dependencies
RUN pip install -r requirements.txt
RUN #pip install --no-cache-dir -r requirements.txt


# Configure RPC server
EXPOSE $XMLRPC_SERVER_PORT
ENV LOAD_CARLA_EGG="True"
ENV CARLA_EGG_DIR=/usr/lib/carla

# Start the server
WORKDIR /usr/src/sensorlib/test/integration
RUN python3 test_api.py

#RUN python3 CarlaCDASimAdapter.py \
#    --carla-host $CARLA_HOST \
#    --carla-port $CARLA_PORT \
#    --xmlrpc-server-host $XMLRPC_SERVER_HOST \
#    --xmlrpc-server-port $XMLRPC_SERVER_PORT
