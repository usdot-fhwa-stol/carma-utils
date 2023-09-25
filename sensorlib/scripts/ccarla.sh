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

    rl) set-rendering-level;;
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
}

venv() {

  echo "[-] Activating Python virtual environment"
  source env/bin/activate

}

set-rendering-level() {
  set-carla-rendering.sh $1
}

start-carla() {
  $CARLA_ROOT/CarlaUE4.sh
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
