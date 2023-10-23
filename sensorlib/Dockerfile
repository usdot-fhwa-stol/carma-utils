FROM python

WORKDIR /usr/src/sensorlib

COPY . .

EXPOSE $XMLRPC_SERVER_PORT

RUN pip install --no-cache-dir -r requirements.txt
RUN python3 CarlaCDASimAdapter.py \
    --carla-host $CARLA_HOST \
    --carla-port $CARLA_PORT \
    --xmlrpc-server-host $XMLRPC_SERVER_HOST \
    --xmlrpc-server-port $XMLRPC_SERVER_PORT
