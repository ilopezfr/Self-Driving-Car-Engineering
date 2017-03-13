## Vehicle Detection Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a Linear SVM classifier
* Apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector.
* Normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

--- 
[//]: # (Image References)

[image0]: ./writeup_images/project_video.png "Screenshot to project video"
[image1]: ./writeup_images/hog.png "Hog"
[image2]: ./writeup_images/slide_window.png "Sliding Window"

[image3]: ./writeup_images/output_fp.png "Output"
[image4]: ./writeup_images/output1.png "Output1"
[image5]: ./writeup_images/output2.png "Output2"

[video1]: ./project_video_output.mp4 "Video"

Here's the output of the lane finding pipeline: 
[![alt text][image0]](https://youtu.be/0c_CSzzUDe8)

The pipeline for this project can be found on the accompanying iPython notebook [`./vehicle-detection-project-pipeline.ipynb`](./vehicle-detection-project-pipeline.ipynb)

[`utils.py`](./utils/utils.py) contains the functions and classes used to perform the feature extraction, train the model, apply sliding window search and display the images.


### 1. Feature Extraction - Histogram of Oriented Gradients (HOG)
---

I defined the `ImgFeatExtract` class. This class is instantiated with a list of configuration options for feature extraction. 
The function `feat_extract` does the hard work and performs the following steps:
1. Color-space conversion: allows for conversion to one of the OpenCV supported color spaces.
2. Spatial Features extraction: extract features by collecting color spatial information (per channel) and concatenating to form a feature vector.
3. Color histograms extraction: using `np.histogram`, compute histograms for each image channel, then concatenate them together to form the features. 
4. HOG features extraction: using `skimage.feature.hog()` function, extract features per image channel. Parameter configuration allows performing the feature extraction an individual channel or accross all channels. 
5. Combine all steps above and return the feature vector. 


Different colorspaces and HOG parameters were explored. The final configuration was chosen according to the one which gave the best test-set accuracy from the classifier. In the process, I learned that extracting the HOG features have the most impact on the resulted feature vector. I tested using different color techiniques with not much improvement on the results.

This is the configuration that gave me the highest accuracy results:
feature_config = {
    'color_space' : 'YCrCb',
    'orient' : 12,
    'pix_per_cell' : 8,
    'cell_per_block' : 2,
    'hog_channel' : 'ALL',
    'spatial_size' : (32, 32),
    'hist_bins' : 32,
    'spatial_feat' : True,
    'hist_feat' : True,
    'hog_feat' : True
}

[![alt text][image1]]


### 2. Training the classifier. 
A Linear SVM was chosen as my classifier. SVC provides relatively high accuracy and fast computation, which makes it a good fit for this type of job. 
Here are the steps followed to train the classifier:
1. Read all the positive images (with cars)
2. Read all the negative images (without cars)
3. Instantiate my feature extractor `ImgFeatExtract` with the chosen configuration.
4. Stack the feature vectors (HOG, Spatial, Color Histogram) together and create the labels. 
5. Split data into train and test, 70-30. 
6. Instantiate a `LinearSVC` , fit it to the features and test its accuracy. 

Without tuning the hyperparameters of LinearSVM and using the default ones, I achieved an accuracy of 99.36%, which is pretty goood for the model.


### 3. Sliding Window Search
In this step is where we detect the vehicles by applying sliding window search. This algorithm is implemented in 2-steps: 
1. Pyramid search or Scale search: We slide the windows across the image at different scales. This is good to detect objects in varying sizes, like cars driving on the road that are at different distances from me.  This method is applied using `get_sliding_windows` in  `process` of class ObjectDetector. 
This method in-turn calls method `get_sliding_windows` passing it a different scale (or image size) on each iteration. get_sliding_windows returns an array of window points (rectangles). The windows are slid across the image with an amount of overlap that is configured--The pipeline I used 80% of overlap. Also the pipeline ignores the top of the image that doesn't corresponds with the road. 
2. Slide the windows. Here's we pass a list of windows previously computed to the `find_objects()` function and loop thought all them. Each one of them is run through the feature extraction and classificaiton pipeline. Finally, it returns a list with the prediction hits.

These are the results applying the sliding window search method to the test images.
[![alt text][image2]

From left to right:
1. Output of the sliding windows algorithm, as implemented in the find_objects method. 
This shows the sliding windows that had positive detections. 

2.Blob extraction. The problem with HOG feature extraction is the number of false positives it can return. To deal with this, I built up a "heatmap" of our sliding window detections. This is done by creating a 2D array with the same dimensions of the image. 

3. Heatmap. The brighter pixels indicate the concentration of sliding windows. I applied threshold mask to the heat to perform blob detection.
This segregates our found sliding windows into detected individual objects as shown next.

4. Heatmap converted to bounding boxes. 

### 4. Video Implementation of the pipeline
Here's a [link to my video result](./output_images/project_video_output.mp4)

Some frames extracted from the video:

[![alt text][image3]
Here we can see one example of a False Positive. 

[![alt text][image4]
It properly detects and tracks the 2 vehicles when the enter into the visual field.

[![alt text][image2]

### Discussion

Much of the time was spent on iterating on different parameter values for the Image Feature Extraction configuration and training on the test images. This time I didn't use a GPU so every time I had to run the training pipeline it took some time. 

Another issue I found was the high number of false positives when applying HOG feature extraction. However, I was able to reduce this number by tuning the blob detection and setting up thresholds to the heat map during the sliding window search. 

As per improvements on the output--mainly reduction of false positives-- one thing that could help would be using Kalman Filtera and finding a way to detect the horizon and automatically mask out only those places where a vehicle can show up.
