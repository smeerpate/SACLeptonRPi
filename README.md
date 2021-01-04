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

## Measuring method
An estimation of the test person's core temperature is done by measuring the forehead temperature. This is done using an uncooled microbolometer. Based to the article *Investigation of the Impact of Infrared Sensors on Core Body Temperature Monitoring by Comparing Measurement Sites* (https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7284737/) by Hsuan-Yu Chen, Andrew Chen and Chiachung Chen.


  * Detect a face using a convolutional neural network implemented in openCV
  * Wait until the face is close enough by measuring its width
  * Set the region of interest (ROI) to the forehead region. This is currently done by taking the top 1/3rd of the returned face bounding box (room for improvement)
  * Map the region of interest to the Thermal imaging sensor image using an affine transform
  * Perform a flat field calibration (FFC) on the Thermal imaging sensor if the last FFC happened more than 20" ago
  * Set the Flux Linear parameters in the Thermal imaging sensor. This code is currently using the Thermal imaging sensor housing temperature as ambient temperature (also room for improvement)
  * Write the ROI to the Thermal imaging sensor via the CCI
  * Read back temperatures from the Thermal imaging sensor via the CCI
  * Wait to go back to idle state until detected face is gone
  * The maximum temperature is taken as the temperature of the forehead
  
  ### Results
  115 Measurements on different healthy persons resulted in:
  
  Average forehead temperature (degC) | Stddev | Coefficient of variation (CV%) 
  ----------------------------------- | ------ | ------------------------------
  33.324 | 0.560 |1.680
  
