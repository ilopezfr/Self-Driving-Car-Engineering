
## CameraCalibration.py

import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.misc import imresize, imread

CAL_IMAGE_PATH ='camera_cal/calibration*.jpg'
CAL_IMAGE_SIZE = (720, 1280, 3)

def get_calibration(path_to_images):  #9, 6
    """Returns camera calibration matrix using
    chessboard images in given path.
    It assumes image size (720, 1280, 3)
    and either 6x9 or 5x9 corners
    """
    obj_points = []  # 3d point in real world space
    img_points = []  # 2d points in image plane.

    images = glob.glob(path_to_images)
    #cal_images = np.zeros((len(images), *CAL_IMAGE_SIZE), dtype=np.uint8)
    total_img_count = len(images)
    #print (total_img_count)
    #exit()


    img_count = 1
    fig = plt.figure()
    for fname in images:
        img = cv2.imread(fname)
        nx, ny = 6, 9
        # resize image if it's different shape than (720, 1280, 3)
        if img.shape[0] != CAL_IMAGE_SIZE[0] or img.shape[1] != CAL_IMAGE_SIZE[1]:
            img = imresize(img, CAL_IMAGE_SIZE)

        #gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(img, (nx, ny), None) #None?

        objp = np.zeros((nx * ny, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        # include special case of cheesboard image with 5x9
        if not ret:
            nx, ny = 5,9
            objp = np.zeros((nx * ny, 3), np.float32)
            objp[:, :2] = np.mgrid[0: nx, 0: ny].T.reshape(-1, 2)
            ret, corners = cv2.findChessboardCorners(img, (nx, ny))

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

            ax = fig.add_subplot(math.ceil(total_img_count / 2), 2, img_count)
            img_w_corners = cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
            img_w_corners = cv2.cvtColor(img_w_corners, cv2.COLOR_BGR2RGB)
            plt.title(fname)
            ax.imshow(img)
            ax.axis('off')
            #cal_images[idx] = img

            img_count += 1

    print("%s/%s camera calibration images processed." % (img_count, len(images)))

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, CAL_IMAGE_SIZE[:-1], None, None)
    calibration = {'obj_points': obj_points,
                   'img_points': img_points,
                   #'cal_images': cal_images,
                   'mtx': mtx,
                   'dist': dist,
                   'rvecs': rvecs,
                   'tvecs': tvecs}
    return calibration, fig

def undistort(img, mtx, dist):
    """Returns undistorted image
    """
    return cv2.undistort(img, mtx, dist)



