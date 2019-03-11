<<<<<<< HEAD
# Project 3: Use Deep Learning to Clone Driving Behavior

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This repository contains starting files for P3, Behavioral Cloning.

In this project, you will use what you've learned about deep neural networks and convolutional neural networks to clone driving behavior. You will train, validate and test a model using Keras. The model will output a steering angle to an autonomous vehicle.

We have provided a simulator where you can steer a car around a track for data collection. You'll use image data and steering angles to train a neural network and then use this model to drive the car autonomously around the track.

We also want you to create a detailed writeup of the project. Check out the [writeup template](https://github.com/udacity/CarND-Behavioral-Cloning-P3/blob/master/writeup_template.md) for this project and use it as a starting point for creating your own writeup. The writeup can be either a markdown file or a pdf document.

To meet specifications, the project will require submitting five files: 
* model.py (script used to create and train the model)
* drive.py (script to drive the car - feel free to modify this file)
* model.h5 (a trained Keras model)
* a report writeup file (either markdown or pdf)
* video.mp4 (a video recording of your vehicle driving autonomously around the track for at least one full lap)

This README file describes how to output the video in the "Details About Files In This Directory" section.

Creating a Great Writeup
---
A great writeup should include the [rubric points](https://review.udacity.com/#!/rubrics/432/view) as well as your description of how you addressed each point.  You should include a detailed description of the code used (with line-number references and code snippets where necessary), and links to other supporting documents or external references.  You should include images in your writeup to demonstrate how your code works with examples.  

All that said, please be concise!  We're not looking for you to write a book here, just a brief description of how you passed each rubric point, and references to the relevant code :). 

You're not required to use markdown for your writeup.  If you use another method please just submit a pdf of your writeup.

The Project
---
The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior 
* Design, train and validate a model that predicts a steering angle from image data
* Use the model to drive the vehicle autonomously around the first track in the simulator. The vehicle should remain on the road for an entire loop around the track.
* Summarize the results with a written report

### Dependencies
This lab requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

The following resources can be found in this github repository:
* drive.py
* video.py
* writeup_template.md

The simulator can be downloaded from the classroom. In the classroom, we have also provided sample data that you can optionally use to help train your model.

## Details About Files In This Directory

### `drive.py`

Usage of `drive.py` requires you have saved the trained model as an h5 file, i.e. `model.h5`. See the [Keras documentation](https://keras.io/getting-started/faq/#how-can-i-save-a-keras-model) for how to create this file using the following command:
```sh
model.save(filepath)
```

Once the model has been saved, it can be used with drive.py using this command:

```sh
python drive.py model.h5
```

The above command will load the trained model and use the model to make predictions on individual images in real-time and send the predicted angle back to the server via a websocket connection.

Note: There is known local system's setting issue with replacing "," with "." when using drive.py. When this happens it can make predicted steering values clipped to max/min values. If this occurs, a known fix for this is to add "export LANG=en_US.utf8" to the bashrc file.

#### Saving a video of the autonomous agent

```sh
python drive.py model.h5 run1
```

The fourth argument `run1` is the directory to save the images seen by the agent to. If the directory already exists it'll be overwritten.

```sh
ls run1

[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_424.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_451.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_477.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_528.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_573.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_618.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_697.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_723.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_749.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_817.jpg
...
```

The image file name is a timestamp when the image image was seen. This information is used by `video.py` to create a chronological video of the agent driving.

### `video.py`

```sh
python video.py run1
```

Create a video based on images found in the `run1` directory. The name of the video will be name of the directory following by `'.mp4'`, so, in this case the video will be `run1.mp4`.

Optionally one can specify the FPS (frames per second) of the video:

```sh
python video.py run1 --fps 48
```

The video will run at 48 FPS. The default FPS is 60.

#### Why create a video

1. It's been noted the simulator might perform differently based on the hardware. So if your model drives succesfully on your machine it might not on another machine (your reviewer). Saving a video is a solid backup in case this happens.
2. You could slightly alter the code in `drive.py` and/or `video.py` to create a video of what your model sees after the image is processed (may be helpful for debugging).
=======
## Advanced Lane Finding Project
---
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The code and results to generate this report can be found on the notebook: advanced-lane-project-pipeline [add link]

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image0]: ./writeup_images/output1.png "Undistorted"
[image1]: ./writeup_images/undistorted_output.png "Undistorted"
[image2]: ./writeup_images/undistorted_road.png "Undistorted Road"
[image3]: ./writeup_images/threshold.png "Thesholded Road"
[image4]: ./writeup_images/warped_lines.png "Warped Lines"
[image5]: ./writeup_images/lane_detected.png "Lane Detected"
[image6]: ./writeup_images/curvature.png "Curvature"
[image7]: ./writeup_images/plotting_back.png "Plotting"
[image8]: ./writeup_images/output1.png "Output"
[image9]: ./writeup_images/output2.png "Output"
[image10]: ./writeup_images/output3.png "Output"
[image11]: ./writeup_images/output4.png "Output"
[video1]: ./project_video_out.mp4 "Video"

Here's the output of the lane finding pipeline: 
[![alt text][image0]](https://youtu.be/0c_CSzzUDe8)

The code for this project can be found at [`./advanced-lane-project-pipeline.ipynb`](./advanced-lane-project-pipeline.ipynb). 


###Camera Calibration
---
To perform the camera calibration, I used the chessboard images located at `images/calibration` and run their grayscaled versions through `cv2.findChessboardCorners`. I make the assumptions that:
- chessboard is fixed at z=0 plane.
- image size (720, 1280, 3)  
- chessboards contain 6x9 or 5x9 corners. 

The first assumption implies that object points are the same for each calibration image. Then I run the function cv2.calibrateCamera that detects corners (`imgpoints`) and chessboard coordinates (`objpoints`) for each image and returns the distortion coefficients.
The functions used in this step, `get_calibration()` and `undistort()` can be found in the utils folder.  

Below is a side-by-side of an undistorted image and its original.
![alt text][image1]

###Pipeline (single images)
---
####1. Camera distortion correction
Below is an example of a before and after distortion-corrected test image. I used `undistort` function--which can be found in utils-- with the parameters camera matrix (`mtx`) to transform 3D to 2D and the distrotion coefficients (`dist`) obtained during the camera calibration step. 

![alt text][image2]

####2.Image Thresholding
Then I used a combination of color and gradient thresholds to generate a binary image. I used the function `threshold()`, which can be found in `utils.py`.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps are in function threshold_binary() in notebook cell 11.

First I converted the image to HLS color space, and grabbed the V channel. I then thresholded the image using different pixel identities to separate white and yellow masks.

Additionally we use the OpenCV Sobel function to take the gradient of the image in the X direction. By taking the gradient in the X direction we can better pick out vertical features in the image (such as lane lines).

Finally I merged these two thresholded images. Below is an example of my output for this step showing both the images described above. One is for debugging purposes where we stack the two images on top of each other (Combination of Masks) - enabling us to see in one image how each technique is contributing. The other (Thresholded Binary) is simply a bitwise-or between the two images. I used this one to find the lane lines.

![alt text][image3]

To find lane lines and their curvature, I followed these steps:
- performed a perspective transformation on the image to get a top-down or birds-eye-view of the road.
- Then used an histogram along the bottom half of the image to find the lane lanes.
- Used a sliding window to iteratively move up the image finiding the lane lines until the top of the image. 
- Fit a second order polynomial to find the lane line pixels. 

####3. Perspective Transformation

The code for my perspective transform uses OpenCV's `getPerspectiveTransform()` and `warpPerspective()` functions and is based on a set of source and destination images points. The source and destination points chosen are the following: 

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 588, 446      | 200, 0        | 
| 691, 446      | 1080, 0       |
| 1126, 673     | 1080, 720     |
| 153, 673      | 200, 72       |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image. Below is a sample image where binary thresholding and perspective transform applied. We can appreciate the line lines, althought with some noise: 

![alt text][image4]

####4. Lane Detection
I added up the pixel values along each column in the image. In my thresholded binary image, pixels are either 0 or 1, so the two most prominent peaks in this histogram are good indicators of the x-position of the base of the lane lines. I then used that as a starting point for my search for lines with a margin of 100px, and considered 1/9th of the image starting from bottom.

Once one of the first frames was processed, I used the last known line location to restrict the search for new lane pixels. The code to perform the lane detection  the same can be found in the `advanced-lane-project-pipeline.ipynb` notebook. These are the results:

![alt text][image5]

####5. Measuring Curvature.

The radious of curvature for each line is calculated using the following relationship between pixel coordinates and real world coordinates:

![alt text][image6]

These are the steps followed:
- Convert from pixels to meters
- Fit polynomials to the left and right lane line points.
- Calculate the curvature as per the equation above.
- Calculate the lane deviation from the center (between lane lines and assuming the camera is in the center of the car).

####6. Plot back down onto the road.

Plotting the identified lanes back on the original image of the road, this is how it looks: 
![alt text][image7]

---

###Pipeline (video)
---

Here's a [link to my video result](./project_video_out.mp4)
For implemetation details check [`./utils/pipeline.py`](./utils/pipeline.py) and [`./utils/line.py`](./utils/line.py).

Some frames from the output video:
![alt text][image8]
![alt text][image9]
![alt text][image10]
![alt text][image11]

---

###Discussion
---
The pipeline performs reasonably good on the project video, even with the limited parameter tuning performed on the thresholding and smoothing steps. However it does not that well on the challenge videos. The reason is that the creation of binary image doesn't work well detecting the lane under the varied brightness situations on the road surface encountered on these videos. 
I could build a more robust pipeline if I try using some methods like:
- contrast-limited adaptive histogram equalization (CLAHE) to account for the varied brightness conditions.
- Result smoothing. Use a weighted average or smoothing such as a first order filter response, i.e. coeffs = 0.95*coeff~prev+ 0.05 coeff.
- Colour spaces. Investigate other colour space and their channels to see which still shows the lanes the best over the concrete sections of road.

>>>>>>> CarND-Advanced-Lane-Lines/master
