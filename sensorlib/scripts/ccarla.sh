#!/bin/bash



# ------------------------------------------------------------------------------
# Description
# ------------------------------------------------------------------------------





# Reference
# - [Rendering toggle](https://carla.readthedocs.io/en/latest/adv_rendering_options/#off-screen-mode)
# - [Headless mode](https://carla.readthedocs.io/en/0.9.5/carla_headless/)

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------

VENV_ROOT=$HOME/.venv
DEFAULT_VENV_NAME="carla"
CARLA_ROOT=$HOME/carla




# ------------------------------------------------------------------------------
# Process Arguments
# ------------------------------------------------------------------------------

main() {

  # Handle only the first command passed into the script
  case $1 in

    egg) load-egg;;

    mkvenv) mkvenv;;
    venv) venv;;

    start) start-carla;;

    help) print_help;;
    --help) print_help;;
    *) print_help;;

  esac;

  # Exit with code
  exit 0
}


# ------------------------------------------------------------------------------
# Functions
# ------------------------------------------------------------------------------

load-egg() {
  CARLA_EGG_FILE=""
  # ...
  echo "CARLA .egg file:  $CARLA_EGG_FILE"
}

mkvenv() {
  mkdir $VENV_ROOT
  VENV_NAME=$DEFAULT_VENV_NAME
  echo "[-] Making Python virtual environment \"$VENV_NAME\""
  python3 -m venv $VENV_ROOT/$VENV_NAME
  echo "[-] Configuring virtual environment"
  source $VENV_ROOT/$VENV_NAME/bin/activate
  pip3 install --upgrade pip wheel
  python3 -m pip install -r requirements.txt

#  echo "[-] Installing CARLA dependencies"
#  pip3 install --upgrade pip wheel
#  pip3 install carla==0.9.5

  deactivate
}

venv() {
  VENV_NAME=$DEFAULT_VENV_NAME
  echo "[-] Activating Python virtual environment"
  source $VENV_ROOT/$VENV_NAME/bin/activate
}

set-rendering-level() {
  echo "Not implemented"
}

start-carla() {
  if [ -z "$1" ]; then
    $CARLA_ROOT/CarlaUE4.sh
  else
    set-carla-rendering.sh $1
  fi
  #$CARLA_ROOT/CarlaUE4.sh -opengl -carla-server -carla-no-window
#  $CARLA_ROOT/CarlaUE4.sh -carla-server -carla-no-graphics -opengl -ResX 1280 -ResY 720 -benchmark -fps 120
}

print_help() {
cat <<'_HELP_TEXT'

ccarla <command> <args>

CARLA configuration utility.

Commands:

  egg               Load the CARLA Python .egg file.

  mkvenv <name>     Make a named Python virtual environment containing CARLA and related dependencies.
  venv <name>       Load the Python virtual environment.

  rl <level>        Set the CARLA rendering level.
                      0: Disable all rendering computations.
                      1: Disable normal rendering and show simplified overhead view.
                      2: Enable full local rendering.
                      3: Configure as a headless server. Rendering is performed locally but only available to view through remote connection.

  start             Start CARLA.

  help              Print this help information.

_HELP_TEXT
}


# ------------------------------------------------------------------------------
# Run
# ------------------------------------------------------------------------------

# Execute the script with forward-declared functions
main $@
