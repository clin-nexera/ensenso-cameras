# Classes For Interacting With IDS Cameras

## Requirements

1. Install the Ensenso SDK: https://www.optonic.com/en/support/download/ensenso-sdk/
2. Install the UEye Driver (maybe): https://www.optonic.com/en/support/download/ensenso-sdk/
3. pip install -r requirements.txt
    -   pip install nxlib
    -   pip install gpib_ctypes
4. Copy and paste everything into the constant.py file into the nxlib/constants.py file

## Serial Numbers
- RGB By Window: "4104140356"
- Ensenso By Window: "216866"
- Ensenso By Door: "216865"


## Classes

### EnsensoCamera
- open_camera : open camera
- close_camera : close camera
- capture : capture depth and texture images
- save_texture_img(filename="texture_img.png") : save the texture image