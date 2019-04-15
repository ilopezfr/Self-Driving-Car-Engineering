# utils.py


## CameraCalibration

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


## Thresholding
import cv2
import numpy as np
import pickle


# Define a function that takes an image, gradient orientation,
# and threshold min / max values.
def abs_sobel_thresh(img, orient='x', thresh_min=0, thresh_max=255):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Apply x or y gradient with the OpenCV Sobel() function
    # and take the absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    # Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    # Create a copy and apply the threshold
    binary_output = np.zeros_like(scaled_sobel)
    # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    binary_output[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
    # Return the result
    return binary_output


# Apply the sobel operator to an image and identify pixels where the gradient 
# of the image falls within a specified threshold range.
def dir_thres(img, sobel_kernel=15, thresh=(0.7, 1.2)):
    # Convert to Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    # Calculate the x and y gradients
    sobelx = cv2.Sobel(gray[:,:,2], cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray[:,:,2], cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the x and y gradients
    # and use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient
    abs_grad_dir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    # Rescale back to 8 bit integer
    # scaled_sobel = np.uint8(255*abs_grad_dir/np.max(abs_grad_dir))
    # aCreate a copy and apply the threshold
    binary_output =  np.zeros_like(abs_grad_dir)
    binary_output[(abs_grad_dir >= thresh[0]) & (abs_grad_dir <= thresh[1])] = 1
    # Return the binary image
    return binary_output

# Define a function to return the magnitude of the gradient
# for a given sobel kernel size and threshold values
def mag_thresh(img, sobel_kernel=9, mag_thresh=(30, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    # Take both Sobel x and y gradients
    sobelx = cv2.Sobel(gray[:,:,2], cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray[:,:,2], cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the gradient magnitude
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    # Rescale to 8 bit
    scale_factor = np.max(gradmag)/255 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    # Return the binary image
    return binary_output


def threshold(img, color=False, mag_dir_thresh=False):
    """Threshhold image on saturation channel and 
    using magnitude gradient"""
    img = np.copy(img)
    
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    
    
    ## White Color
    lower_white = np.array([0,210,0], dtype=np.uint8)
    upper_white = np.array([255,255,255], dtype=np.uint8)
    white_mask = cv2.inRange(hls, lower_white, upper_white)
    
    ## Yellow Color
    lower_yellow = np.array([15,0,100], dtype=np.uint8)
    upper_yellow = np.array([40,220,255], dtype=np.uint8)
    yellow_mask = cv2.inRange(hls, lower_yellow, upper_yellow)  
    
    combined_binary = np.zeros_like(white_mask)
    
    # Dir Mag Threshold
    if mag_dir_thresh:
        dir_mask = dir_thres(img)
        mag_mask = mag_thresh(img)
        combined_binary[((dir_mask == 1) & (mag_mask == 1))] = 255
        
    if color:
        return np.dstack((white_mask, yellow_mask, combined_binary))
    
    else:
        combined_binary[((white_mask == 255) | (yellow_mask == 255))] = 255
        combined_binary[(combined_binary == 255)] = 1
        return combined_binary

