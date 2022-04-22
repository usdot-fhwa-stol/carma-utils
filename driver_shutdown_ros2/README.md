# driver_shutdown_ros2

Simple node which subscribes to ```/system_alert``` and will shut itself down if it receives a SHUTDOWN alert. 
This is meant to allow for driver launch files to be exited when running in separate containers by marking this node as required.
