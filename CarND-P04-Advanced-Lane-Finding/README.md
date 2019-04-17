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


### Camera Calibration
---
To perform the camera calibration, I used the chessboard images located at `images/calibration` and run their grayscaled versions through `cv2.findChessboardCorners`. I make the assumptions that:
- chessboard is fixed at z=0 plane.
- image size (720, 1280, 3)  
- chessboards contain 6x9 or 5x9 corners. 

The first assumption implies that object points are the same for each calibration image. Then I run the function cv2.calibrateCamera that detects corners (`imgpoints`) and chessboard coordinates (`objpoints`) for each image and returns the distortion coefficients.
The functions used in this step, `get_calibration()` and `undistort()` can be found in the utils folder.  

Below is a side-by-side of an undistorted image and its original.
![alt text][image1]

### Pipeline (single images)
---
#### 1. Camera distortion correction
Below is an example of a before and after distortion-corrected test image. I used `undistort` function--which can be found in utils-- with the parameters camera matrix (`mtx`) to transform 3D to 2D and the distrotion coefficients (`dist`) obtained during the camera calibration step. 

![alt text][image2]

#### 2.Image Thresholding
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

#### 3. Perspective Transformation

The code for my perspective transform uses OpenCV's `getPerspectiveTransform()` and `warpPerspective()` functions and is based on a set of source and destination images points. The source and destination points chosen are the following: 

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 588, 446      | 200, 0        | 
| 691, 446      | 1080, 0       |
| 1126, 673     | 1080, 720     |
| 153, 673      | 200, 72       |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image. Below is a sample image where binary thresholding and perspective transform applied. We can appreciate the line lines, althought with some noise: 

![alt text][image4]

#### 4. Lane Detection
I added up the pixel values along each column in the image. In my thresholded binary image, pixels are either 0 or 1, so the two most prominent peaks in this histogram are good indicators of the x-position of the base of the lane lines. I then used that as a starting point for my search for lines with a margin of 100px, and considered 1/9th of the image starting from bottom.

Once one of the first frames was processed, I used the last known line location to restrict the search for new lane pixels. The code to perform the lane detection  the same can be found in the `advanced-lane-project-pipeline.ipynb` notebook. These are the results:

![alt text][image5]

#### 5. Measuring Curvature.

The radious of curvature for each line is calculated using the following relationship between pixel coordinates and real world coordinates:

![alt text][image6]

These are the steps followed:
- Convert from pixels to meters
- Fit polynomials to the left and right lane line points.
- Calculate the curvature as per the equation above.
- Calculate the lane deviation from the center (between lane lines and assuming the camera is in the center of the car).

#### 6. Plot back down onto the road.

Plotting the identified lanes back on the original image of the road, this is how it looks: 
![alt text][image7]

---

### Pipeline (video)
---

Here's a [link to my video result](./project_video_out.mp4)
For implemetation details check [`./utils/pipeline.py`](./utils/pipeline.py) and [`./utils/line.py`](./utils/line.py).

Some frames from the output video:
![alt text][image8]
![alt text][image9]
![alt text][image10]
![alt text][image11]

---

### Discussion
---
The pipeline performs reasonably good on the project video, even with the limited parameter tuning performed on the thresholding and smoothing steps. However it does not that well on the challenge videos. The reason is that the creation of binary image doesn't work well detecting the lane under the varied brightness situations on the road surface encountered on these videos. 
I could build a more robust pipeline if I try using some methods like:
- contrast-limited adaptive histogram equalization (CLAHE) to account for the varied brightness conditions.
- Result smoothing. Use a weighted average or smoothing such as a first order filter response, i.e. coeffs = 0.95*coeff~prev+ 0.05 coeff.
- Colour spaces. Investigate other colour space and their channels to see which still shows the lanes the best over the concrete sections of road