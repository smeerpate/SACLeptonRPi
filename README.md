# SACLeptonRPi
Flir lepton 80x60 sensor and RPi 3B symbiosys

Neural network installation: https://github.com/tensorflow/tensorflow/issues/36141?fbclid=IwAR0d4MILHB_GreDxLU-YguQiNk__Gjb3_gvgaCV7GI_SVz65TxI18XAMd-Q

NSC2 installation: https://www.pyimagesearch.com/2019/04/08/openvino-opencv-and-movidius-ncs-on-the-raspberry-pi/

sudo addgroup users

Service creation:

cd /lib/systemd/system

sudo nano SACLeptonRPi.service

To test if it works: sudo systemctl start SACLeptonRPi.service

If so:

sudo systemctl daemon-reload

sudo systemctl stop SACLeptonRPi.service

sudo systemctl enable SACLeptonRPi.service

## Mechanics
* `/home/pi/SACLeptonRPi/SACLeptonRPi.sh`
  1. runs openVINO setupvars script (under `~/openvino/bin/setupvars.sh`)
  1. starts `~/SACLeptonRPi/Main.py`
* `~/SACLeptonRPi/Main.py` starts openGLes display thread `/home/pi/SACLeptonRPi/SACDisplayMixer/OGLESSimpleImageWithIPC`
