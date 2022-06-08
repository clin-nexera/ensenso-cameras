import argparse

from nxlib.context import NxLib, MonoCamera
from nxlib.command import NxLibCommand
from nxlib.constants import *

class MonoCam():
    
    def __init__(self, serial):
        self.camera_serial = serial

    def capture_raw(self, filename="raw.png"):
        with NxLib(), MonoCamera("4104140356") as camera:
            camera.capture()
            save_image(filename, camera[ITM_IMAGES][ITM_RAW])

    def capture_rectified(self, filename="rectified.png"):
        with NxLib(), MonoCamera(self.camera_serial) as camera:
            camera.capture()
            camera.rectify()
            save_image(filename, camera[ITM_IMAGES][ITM_RECTIFIED])


def save_image(filename, item):
    with NxLibCommand(CMD_SAVE_IMAGE) as cmd:
        cmd.parameters()[ITM_NODE] = item.path
        cmd.parameters()[ITM_FILENAME] = filename
        cmd.execute()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("serial", type=str,
                        help="the serial of the stereo camera to open")
    args = parser.parse_args()

    camera_serial = args.serial
    # instantiate camera
    mono_camera = MonoCam(camera_serial)
    # capture and save a raw image
    mono_camera.capture_raw("raw_img.png")
    # capture and save a rectified image
    mono_camera.capture_rectified("rectified_img.png")

