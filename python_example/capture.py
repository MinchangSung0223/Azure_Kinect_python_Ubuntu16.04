import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import argparse
from PIL import Image
import signal
from matplotlib import pyplot as plt
import time
import numpy as np
import cv2
import threading
from ctypes import cdll
import open3d as o3d
import ctypes
from numpy.ctypeslib import ndpointer
lib = cdll.LoadLibrary('./viewer_opengl.so')
st = lib.Foo_start
t0 = threading.Thread(target=st)
t0.start()
end = lib.Foo_end
dataread =lib.Foo_dataread
dataread_color =lib.Foo_dataread_color
dataread_depth =lib.Foo_dataread_depth
dataread_get_pointcloud_xyz = lib.Foo_get_pointcloud_xyz
dataread_get_pointcloud_rgb = lib.Foo_get_pointcloud_rgb

dataread_color_to_depth =lib.Foo_dataread_color_to_depth
dataread.restype = ndpointer(dtype=ctypes.c_uint16, shape=(720,1280))
dataread_color.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720,1280,4))
dataread_depth.restype = ndpointer(dtype=ctypes.c_uint16, shape=(512,512))#ctypes.POINTE
dataread_color_to_depth.restype = ndpointer(dtype=ctypes.c_uint8, shape=(512,512,4))
dataread_get_pointcloud_xyz.restype = ndpointer(dtype=ctypes.c_int16, shape=(720*1280,3))
dataread_get_pointcloud_rgb.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720*1280,3))

#convert_2d_3d = lib.Foo_convert_2d_3d
#convert_2d_3d.restype = ndpointer(dtype=ctypes.c_float, shape=(3))#ctypes.POINTE
def signal_handler(signal, frame):
  # Press Ctrl + \
  end()
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
def detect_img():

    n = 0
    while True:
          color_data  =np.array(dataread_color(),dtype=np.uint8)
          depth_data  =np.array(dataread_depth(),dtype=np.uint16)
          color_image_to_depth_camera_data  =np.array(dataread_color_to_depth(),dtype=np.uint16)
          depth_image_to_color_camera_data  =np.array(dataread(),dtype=np.uint8)
          pointCloud_xyz = np.array(dataread_get_pointcloud_xyz(),dtype=np.int16)
          pointCloud_rgb = np.array(np.array(dataread_get_pointcloud_rgb(),dtype=np.uint8),np.float)/255.0
          #print(np.max(pointCloud_rgb))
          xyz = np.reshape(pointCloud_xyz,(720*1280,3))
          rgb = np.reshape(pointCloud_rgb,(720*1280,3))
          #print(np.mean(rgb))
          #pcd = o3d.geometry.PointCloud()
         # pcd.points = o3d.utility.Vector3dVector(xyz)
          #pcd.colors = o3d.utility.Vector3dVector(rgb)

          #o3d.io.write_point_cloud("sync.ply", pcd)
          color_img = color_data.copy()
          depth_img = cv2.convertScaleAbs(depth_data)
          color_image_to_depth_camera_img =  cv2.convertScaleAbs(color_image_to_depth_camera_data)
          depth_image_to_color_camera_img =  cv2.convertScaleAbs(depth_image_to_color_camera_data)

          cv2.imshow("Color", color_data)
          cv2.imshow("Depth", depth_img)

          cv2.imshow("Color_to_Depth", color_image_to_depth_camera_img)
          cv2.imshow("Depth_to_Color", depth_image_to_color_camera_img)
          cv2.waitKey(1)

FLAGS = None

if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()

