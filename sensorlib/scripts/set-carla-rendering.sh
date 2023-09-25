#!/bin/bash
# Set CARLA rendering option.
#
# Usage:
#  ./set_rendering_option.sh  <rendering-level>
#  ./set_rendering_option.sh 1
#  ./set_rendering_option.sh 2
#  ./set_rendering_option.sh 3
#
# Rendering level:
#   0: Disable all rendering computations.
#   1: Disable normal rendering and show simplified overhead view.
#   2: Enable full local rendering.
#   3: Configure as a headless server. Rendering is performed locally but only available to view through remote connection.
#
# Reference
# - [Rendering toggle](https://carla.readthedocs.io/en/latest/adv_rendering_options/#off-screen-mode)
# - [Headless mode](https://carla.readthedocs.io/en/0.9.5/carla_headless/)

CARLA_ROOT=$HOME/carla

RENDERING_LEVEL=$1

echo "Setting CARLA rendering level $RENDERING_LEVEL"
echo

if [ "$RENDERING_LEVEL" == "0" ]; then

  cd $CARLA_ROOT/PythonAPI/util
  python3 config.py --no-rendering
  cd -

elif [ "$RENDERING_LEVEL" == "1" ]; then

  cd $CARLA_ROOT/PythonAPI/examples
  python3 no_rendering_mode.py
  cd -

elif [ "$RENDERING_LEVEL" == "2" ]; then

  cd $CARLA_ROOT/PythonAPI/util
  python3 config.py --rendering
  cd -

elif [ "$RENDERING_LEVEL" == "3" ]; then

  echo "Not implemented"

  #  sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024
  #  sudo nohup Xorg :7 &
  #  /opt/TurboVNC/bin/vncserver :8
  #  DISPLAY=:8 vglrun -d :7.0 glxinfo
  #  DISPLAY=:8 vglrun -d :7.<gpu_number> $CARLA_PATH/CarlaUE4/Binaries/Linux/CarlaUE4

fi
