import numpy as np
np.set_printoptions(threshold=np.nan)
import ctypes
import colorsys

from collections import deque

import urllib.request
import urllib.error

import cv2

from pykinect2.PyKinectRuntime import *
from pykinect2.PyKinectV2 import *

from time import sleep

import matplotlib
#matplotlib.use('TkAgg')
#from matplotlib import pyplot

fire_ij = None
vectors = []
car_coord = deque()
car_origin_ij = None

WINDOW_SIZE = 15

COLOR_HSV_LOWER = [110, 80, 80]
COLOR_HSV_UPPER = [180, 255, 255]

cmd_fire = 0


def send_cmd(distance, angle, fire):
    try:
        urllib.request.urlopen(f'http://192.168.178.36:8080/update_cmd?distance={distance}&angle={angle}&fire={fire}')
    except :
        pass


def find_center(frame, window_size=WINDOW_SIZE, threshold=0.9):
    threshold = threshold * window_size * window_size
    window = np.ones((window_size, window_size))
    height, width = frame.shape[:2]
    for i in range(250, height - window_size, 10):
        for j in range(250, width - window_size, 10):
            if np.sum(frame[i:i + window_size, j:j + window_size] * window) > threshold:
                if len(vectors) == 1 and abs(i - car_origin_ij[0]) < 5*window_size and abs(j - car_origin_ij[1]) < 5*window_size:
                    continue
                elif fire_ij and abs(i - fire_ij[0]) < 5*window_size and abs(j - fire_ij[1]) < 5*window_size:
                    continue
                return i + window_size // 2, j + window_size // 2
    return -1, -1


def ij_to_point(camera_points, i, j, width, height):
    point = camera_points[i * width + j]
    return point.x, point.y, point.z


def get_masked_frame(frame_bgr, lower_hsv, upper_hsv):
    frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower_hsv, upper_hsv = np.array(lower_hsv), np.array(upper_hsv)
    mask = cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
    masked_frame = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask)
    masked_frame = np.array(masked_frame != 0, dtype='int8')
    return masked_frame[:,:,0]


def find_color_center_coord(camera_points, frame_bgr, lower_hsv, upper_hsv):
    global fire_ij
    masked_frame = get_masked_frame(frame_bgr, lower_hsv, upper_hsv)
    i, j = find_center(masked_frame)
    print('color frame coord:', i, j)
    if i == -1:
        return masked_frame, (-1, -1, -1), (-1, -1)
    height, width = masked_frame.shape
    return masked_frame, ij_to_point(camera_points, i, j, width, height), (i, j)


def get_plane_normal(v1, v2, v3):
    v1, v2, v3 = np.array(v1), np.array(v2), np.array(v3)
    normal = np.cross(v1 - v2, v3 - v2)
    normal /= np.sum(normal)
    return normal


def project_vector(v, normal):
    return v - np.dot(v, normal) * normal


def main_loop(kinect):
    global vectors, fire_ij, car_origin_ij, cmd_fire
    width, height = kinect.color_frame_desc.width, kinect.color_frame_desc.height
    depth_width, depth_height = kinect.depth_frame_desc.width, kinect.depth_frame_desc.height

    camera_points = ctypes.POINTER(_CameraSpacePoint)
    camera_points_capacity = ctypes.c_uint(width * height)
    camera_points_type = _CameraSpacePoint * camera_points_capacity.value

    normal_vector = None
    normal_dir = 0
    fire_coord = None
    car_origin_coord = None

    cur_angle = 0
    last_cmd_angle = None

    send_cmd(30, 0, 0)
    #exit()
    
    sleep(3)

    while (1):
        #if not kinect.has_new_depth_frame():
            #continue
        if not kinect.has_new_color_frame():
            continue

        #print('got depth')

        color_frame = kinect.get_last_color_frame()
        depth_frame = kinect.get_last_depth_frame()

        frame_r, frame_g, frame_b = np.array(color_frame[::4]), np.array(color_frame[1::4]), np.array(color_frame[2::4])
        frame_bgr = np.zeros((height, width, 3), dtype='uint8')
        frame_r = frame_r.reshape((height, width))
        frame_g = frame_g.reshape((height, width))
        frame_b = frame_b.reshape((height, width))

        frame_bgr[:, :, 0] = frame_b
        frame_bgr[:, :, 1] = frame_g
        frame_bgr[:, :, 2] = frame_r

        mapper = kinect._mapper
        camera_points = ctypes.cast(camera_points_type(), ctypes.POINTER(_CameraSpacePoint))
        mapper.MapColorFrameToCameraSpace(kinect._depth_frame_data_capacity, kinect._depth_frame_data, camera_points_capacity, camera_points)

        if len(vectors) < 3:
            # green [0, 100, 100], [100, 255, 255]
            masked_frame, (x, y, z), (i, j) = find_color_center_coord(camera_points, frame_bgr, COLOR_HSV_LOWER, COLOR_HSV_UPPER)
            if z != -1 and z != np.inf and z != -np.inf:
                vectors.append(np.array((x,y,z)))
                car_coord.append(np.array((x,y,z)))

                if car_origin_ij is None:  # car station detected
                    print('car origin detected', *vectors[0])
                    car_origin_ij = i, j
                    car_origin_coord = vectors[0]
                
            print('Kinect frame coord:', x, y, z)
            #pyplot.imshow(masked_frame)
            #pyplot.show()



            if len(vectors) == 2:  # fire detected
                fire_coord = vectors[-1]
                fire_ij = i, j
                car_coord[0], car_coord[1] = car_coord[1], car_coord[0]
                send_cmd(15, 0, 0)

        if len(vectors) == 3:
            if normal_vector is None:
                normal_vector = get_plane_normal(*vectors)
                if normal_vector[2] < 0:
                    normal_dir = 1
                else:
                    normal_dir = -1

            v_dest = fire_coord - car_coord[2]
            v_move = car_coord[2] - car_coord[1]
            car_angle = np.arccos(np.dot(v_dest, v_move) / (np.linalg.norm(v_dest) * np.linalg.norm(v_move)))
            if np.isnan(car_angle):
                continue
            if np.dot(np.cross(v_dest, v_move), normal_vector) * normal_dir < 0:
                car_angle *= -1
            cmd_distance, cmd_angle = np.linalg.norm(v_dest)*100, int(car_angle * 180 / np.pi)
            if cmd_angle != last_cmd_angle:
                cur_angle += cmd_angle
                cur_angle = ((cur_angle + 180) % 360) - 180
                last_cmd_angle = cmd_angle

            print(cmd_distance, cur_angle)
 
            if cmd_fire == 0 and cmd_distance < 40 and abs(cmd_angle) < 20:
                cmd_fire = 1
            send_cmd(cmd_distance, cur_angle, cmd_fire)

            camera_points = ctypes.cast(camera_points_type(), ctypes.POINTER(_CameraSpacePoint))
            mapper.MapColorFrameToCameraSpace(kinect._depth_frame_data_capacity, kinect._depth_frame_data, camera_points_capacity, camera_points)
            masked_frame, (x, y, z), (i, j) = find_color_center_coord(camera_points, frame_bgr, COLOR_HSV_LOWER, COLOR_HSV_UPPER)
            if z != -1 and z != np.inf and z != -np.inf:
                new_coord = np.array((x, y, z))
                if np.linalg.norm(new_coord - car_coord[2]) > 0.10:
                    #vectors[2] = np.array((x, y, z))
                    car_coord.append(new_coord)
                    car_coord.popleft()
                    last_cmd_angle = None

            print(fire_coord, car_coord[1], car_coord[2])

        #send_cmd('aaa')
        #print(vectors)

        sleep(3)


if __name__ == '__main__':
    kinect = PyKinectRuntime(FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Body)
    main_loop(kinect)
