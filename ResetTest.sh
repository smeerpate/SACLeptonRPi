#!/bin/bash

echo "Initializing virtualenv"
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh
VIRTUALENVWRAPPER_ENV_BIN_DIR=bin

echo "Going to workon openvino"
workon openvino

echo "Starting SACLeptonRPi/ResetTest.py"
python3 /home/pi/SACLeptonRPi/ResetTest.py
