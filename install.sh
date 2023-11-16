#!/bin/bash

# Force script to exit if an error occurs
set -e

function verify_ready() {
    if [ "$(id -u)" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

verify_ready

# check that klippy python env is present:
KLIPPY_PYTHON_ENV_ACTIVATE="${HOME}/klippy-env/bin/activate"
if [ ! -f "$KLIPPY_PYTHON_ENV_ACTIVATE" ]; then
    echo "$KLIPPY_PYTHON_ENV_ACTIVATE does not exist."
    exit -2
fi

# activate the python environment:
source $KLIPPY_PYTHON_ENV_ACTIVATE

PYTHON_PKGS="pexpect"

for p in $PYTHON_PKGS
do
    if [[ $(pip list | grep -F $p) ]]; then
        echo "Python package '$p' already installed."
    else
        echo "Python package '$p' not installed, installing..."
        pip install --require-virtualenv $p
    fi
done

# execute installation script:
python install.py

# deactivate python enviroment
deactivate
