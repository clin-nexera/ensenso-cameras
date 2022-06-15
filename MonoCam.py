import argparse
import nxlib.api as api

from nxlib.context import NxLib, MonoCamera
from nxlib.command import NxLibCommand,  NxLibException, NxLibItem
from nxlib.constants import *

class MonoCam():
    
    def __init__(self, serial):
        self.camera_serial = serial
        self.raw_img = None
        self.rectified_img = None
        self.is_camera_init_done = False

    def open_camera(self):
        if self.is_camera_init_done == False:
            try:
                # Wait for the cameras to be initialized
                api.initialize()

                # Open the camera with the given serial
                with NxLibCommand(CMD_OPEN) as cmd:
                    cmd.parameters()[ITM_CAMERAS] = self.camera_serial
                    cmd.execute()

                self.is_camera_init_done = True
            except NxLibException as e:
                print(f"An NxLib error occured: Error Text: {e.get_error_text()}")
            except Exception:
                print("An NxLib unrelated error occured:\n")
                raise

    def close_camera(self):
        if self.is_camera_init_done == True:
            try:
                # Closes all open cameras
                with NxLibCommand(CMD_CLOSE) as cmd:
                    cmd.execute()

                self.is_camera_init_done = False
            except NxLibException as e:
                print(f"An NxLib error occured: Error Text: {e.get_error_text()}")
            except Exception:
                print("An NxLib unrelated error occured:\n")
                raise


    def capture_raw(self, filename="raw.png"):
        with NxLib(), MonoCamera(self.camera_serial) as camera:
            camera.capture()
            save_image(filename, camera[ITM_IMAGES][ITM_RAW])
            self.raw_img = camera[ITM_IMAGES][ITM_RAW].get_binary_data()

    def capture_rectified(self, filename="rectified.png"):
        with NxLib(), MonoCamera(self.camera_serial) as camera:
            camera.capture()
            camera.rectify()
            save_image(filename, camera[ITM_IMAGES][ITM_RECTIFIED])
            self.rectified_img = camera[ITM_IMAGES][ITM_RECTIFIED].get_binary_data()


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

