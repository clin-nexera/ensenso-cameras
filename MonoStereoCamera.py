import time
import cv2 as cv
import pickle

import nxlib.api as api
from nxlib import NxLibCommand, NxLibException, NxLibItem, NxLib
from nxlib.constants import *

import EnsensoCamera as ec
import MonoCam as mc


# When using both the mono and stereo cameras
# The cameras are calibrated and linked to each other
# This allows for projection of rgb pixels onto the depth map

class MonoStereoCamera():

    def __init__(self, mono_serial, stereo_serial):
        self.is_camera_init_done = False
        self.depth_image_f32_3d = None
        self.texture_image_u8_4d = None
        self.rendered_point_map_width = 1024
        self.rendered_point_map_height = 1280

        self.mono_serial = mono_serial
        self.stereo_serial = stereo_serial


    def open_camera(self):
        if self.is_camera_init_done == False:
            try:
                api.initialize()

                with NxLibCommand(CMD_OPEN) as cmd:
                    cmd.parameters()[ITM_CAMERAS] = self.stereo_serial
                    cmd.execute()
                     
                    cmd.parameters()[ITM_CAMERAS] = self.mono_serial
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
                
                with NxLibCommand(CMD_CLOSE) as cmd:
                    cmd.execute()
                    self.is_camera_init_done = False
            except NxLibException as e:
                print(f"An NxLib error occured: Error Text: {e.get_error_text()}")
            except Exception:
                print("An NxLib unrelated error occured:\n")
                raise
    
    def log_cameras(self):
        with NxLib():
            cameras = NxLibItem()[ITM_CAMERAS]

            # Print status information for each camera
            print("SerialNo", " " * 8, "Model", " " * 10, "Status")
            for i in range(cameras.count()):
                if not cameras[i][ITM_STATUS].exists():
                    continue
                if check_false(cameras[i][ITM_STATUS][ITM_VALID_IP_ADDRESS]):
                    status = "Invalid Ip"
                elif check_true(cameras[i][ITM_STATUS][ITM_OPEN]):
                    status = "Open"
                elif check_false(cameras[i][ITM_STATUS][ITM_AVAILABLE]):
                    status = "In Use"
                else:
                    status = "Available"
                serial = cameras[i].name()
                model = cameras[i][ITM_MODEL_NAME].as_string()
                print(f"{serial:<17} {model:<16} {status:<16}")

    def set_camera_parameters(self, trigger_delay = 10, auto_exposure = False, exposure = 3):
        NxLibItem()[ITM_CAMERAS][ITM_BY_SERIAL_NO][self.mono_serial][ITM_PARAMETERS][ITM_CAPTURE][ITM_TRIGGER_DELAY] = trigger_delay
        NxLibItem()[ITM_CAMERAS][ITM_BY_SERIAL_NO][self.mono_serial][ITM_PARAMETERS][ITM_CAPTURE][ITM_AUTO_EXPOSURE] = auto_exposure
        NxLibItem()[ITM_CAMERAS][ITM_BY_SERIAL_NO][self.mono_serial][ITM_PARAMETERS][ITM_CAPTURE][ITM_EXPOSURE] = exposure


    def capture(self):
        with NxLibCommand(CMD_CAPTURE) as cmd:
                cmd.execute()

        with NxLibCommand(CMD_RECTIFY_IMAGES) as cmd:
            cmd.execute()

        # Compute the disparity map
        with NxLibCommand(CMD_COMPUTE_DISPARITY_MAP) as cmd:
            cmd.execute()

        translation_0 = NxLibItem()[ITM_CAMERAS][self.stereo_serial][ITM_LINK][ITM_TRANSLATION][0].as_double()
        translation_1 = NxLibItem()[ITM_CAMERAS][self.stereo_serial][ITM_LINK][ITM_TRANSLATION][1].as_double()

        with NxLibCommand(CMD_RENDER_POINT_MAP) as cmd:
            cmd.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][0] = translation_1 # shift target point to camera 
            cmd.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][1] = translation_0
            cmd.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][2] = 0.0
            cmd.parameters()[ITM_SIZE][0] = self.rendered_point_map_width # set width of the images, overriding the default value
            cmd.parameters()[ITM_SIZE][1] = self.rendered_point_map_height # height of the images, overriding the default value
            cmd.parameters()[ITM_FILL_XY_COORDINATES] = True
            
            cmd.execute()

            self.depth_image_f32_3d = NxLibItem()[ITM_IMAGES][ITM_RENDER_POINT_MAP].get_binary_data()
            self.texture_image_u8_4d = NxLibItem()[ITM_IMAGES][ITM_RENDER_POINT_MAP_TEXTURE].get_binary_data()

    def save_texture_img(self, filename="ms_texture_img.png"):
        cv.imwrite(filename, self.texture_image_u8_4d)

    def get_pose(self):
        pose = NxLibItem()[ITM_CAMERAS][ITM_BY_SERIAL_NO][self.mono_serial][ITM_POSE]
        print(pose)

def check_true(item):
    return item.exists() and item.as_bool() is True


def check_false(item):
    return item.exists() and item.as_bool() is False


if __name__ == "__main__":
    mono_stereo = MonoStereoCamera("4104140356", "216866")
    mono_stereo.open_camera()
    mono_stereo.log_cameras()

    for i in range(0,10):
        mono_stereo.set_camera_parameters(trigger_delay=20)
        mono_stereo.capture()
        mono_stereo.save_texture_img(f"test\\test{i}.png")
        time.sleep(0.5)


    mono_stereo.close_camera()
