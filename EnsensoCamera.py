import argparse
import cv2 as cv
import open3d as o3d
import numpy as np

import nxlib.api as api
from nxlib import NxLibCommand, NxLibException, NxLibItem
from nxlib.constants import *


class EnsensoCamera():

    # camera_serial_str = "216866" camera near the window
    # camera_serial_str = "216865" camera near the door
    def __init__(self, serial= "216866"):
        self.camera_serial = serial
        self.is_camera_init_done = False
        self.depth_image_f32_3d = None
        self.texture_image_u8_4d = None
        self.rendered_point_map_width = 1024
        self.rendered_point_map_height = 1280


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


    def capture(self):
        if self.is_camera_init_done == True:
            # Captures with the previous openend camera
            with NxLibCommand(CMD_CAPTURE) as cmd:
                cmd.parameters()[ITM_CAMERAS] = self.camera_serial
                cmd.execute()

            # Rectify the the captures raw images
            with NxLibCommand(CMD_RECTIFY_IMAGES) as cmd:
                cmd.execute()

            # Compute the disparity map
            with NxLibCommand(CMD_COMPUTE_DISPARITY_MAP) as cmd:
                cmd.execute()
            
            # get camera frame position/orientation relative to the robot frame
            rotation_angle = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_ROTATION][ITM_ANGLE].as_double()

            rotation_axis_0 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_ROTATION][ITM_AXIS][0].as_double()
            rotation_axis_1 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_ROTATION][ITM_AXIS][1].as_double()
            rotation_axis_2 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_ROTATION][ITM_AXIS][2].as_double()

            translation_0 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_TRANSLATION][0].as_double()
            translation_1 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_TRANSLATION][1].as_double()
            translation_2 = NxLibItem()[ITM_CAMERAS][self.camera_serial][ITM_LINK][ITM_TRANSLATION][2].as_double()

            # Generate orthographic projection 2D depth and texture images
            rend_point_map = NxLibCommand(CMD_RENDER_POINT_MAP)
            rend_point_map.parameters()[ITM_SIZE][0] = self.rendered_point_map_width # set width of the images, overriding the default value
            rend_point_map.parameters()[ITM_SIZE][1] = self.rendered_point_map_height # height of the images, overriding the default value
            rend_point_map.parameters()[ITM_FILL_XY_COORDINATES] = True
            rend_point_map.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][0] = translation_1 # shift target point to camera 
            rend_point_map.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][1] = translation_0
            rend_point_map.parameters()[ITM_VIEW_POSE][ITM_TRANSLATION][2] = 0.0
            rend_point_map.execute()
            self.depth_image_f32_3d = NxLibItem()[ITM_IMAGES][ITM_RENDER_POINT_MAP].get_binary_data()
            self.texture_image_u8_4d = NxLibItem()[ITM_IMAGES][ITM_RENDER_POINT_MAP_TEXTURE].get_binary_data()


    def save_texture_img(self, filename = "texture_img.png"):
        print("here 2")
        if self.texture_image_u8_4d is not None:
            print("here")
            cv.imwrite(filename, self.texture_image_u8_4d)

        
def ensenso_to_open3d(ensenso_pc):
    point_cloud = o3d.geometry.PointCloud()

    # Reshape from (m x n x 3) to ( (m*n) x 3)
    vector_3d_vector = ensenso_pc.reshape(
        (ensenso_pc.shape[0] * ensenso_pc.shape[1]), ensenso_pc.shape[2])

    # Filter nans: if a row has nan's in it, delete it
    vector_3d_vector = vector_3d_vector[~np.isnan(
        vector_3d_vector).any(axis=1)]
    point_cloud.points = o3d.utility.Vector3dVector(vector_3d_vector)
    return point_cloud



if __name__ == "__main__":
    camera = EnsensoCamera()
    camera.open_camera()
    camera.capture()
    camera.save_texture_img()
    camera.close_camera()