## Vehicle Detection Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
The goals / steps of this project are the following:

Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a Linear SVM classifier
Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector.
Normalize your features and randomize a selection for training and testing.
Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
Estimate a bounding box for vehicles detected.

--- 

The code for this project can be found on the accompanying iPython notebook vehicle-detection.ipynb. 

train.py contains the functions to perform the training
pipeline.py contains the pipeline processing. 


### 1. Feature Extraction - Histogram of Oriented Gradients (HOG)

I've defined the `ImageFeatureExtractor` class. This class is instantiated with a list of configuration options for feature extraction. 
The function `extract_features` does the hard work and performs the following steps:
1. Color-space conversion: allows for conversion to one of the OpenCV supported color spaces.
2. Spatial Features extraction: extract features by collecting color spatial information (per channel) and concatenating to form a feature vector.
3. Color histograms extraction: using `np.histogram`, compute histograms for each image channel, then concatenate them together to form the features. 
4. HOG features extraction: using `skimage.feature.hog()` function, extract features per image channel. Parameter configuration allows performing the feature extraction an individual channel or accross all channels. 
5. Combine all steps above and return the feature vector. 

Out of the steps below, I learned that extracting the HOG features have the most impact on the resulted feature vector. I tested using different color techiniques with not much improvement on the results. 
I believe the parameters could be better fine tuned and improve the results. 

Below is the configuration that gave me better results:
feature_config = {
    'color_space' : 'YCrCb',
    'orient' : 9,
    'pix_per_cell' : 8,
    'cell_per_block' : 2,
    'hog_channel' : 'ALL',
    'spatial_size' : (32, 32),
    'hist_bins' : 32,
    'spatial_feat' : True,
    'hist_feat' : True,
    'hog_feat' : True
}

img: Example HOG features:


### Training the classifier. 
A Linear SVM was chosen as my classifier. SVC provides relatively high accuracy and fast computation, which makes it a good fit for this type of job. 
Here are the steps followed to train the classifier:
1. Read all the positive images (with cars)
2. Read all the negative images (without cars)
3. Instantiate my feature extractor `ImageFeatureExtractor` with the chosen configuration.
4. Stack the feature vectors (HOG, Spatial, Color Histogram) together and create the labels. 
5. Split data into train and test, 70-30. 
6. Instantiate a `LinearSVC` , fit it to the features and test its accuracy. 

Without tuning the hyperparameters of LinearSVM and using the default ones, I achieved an accuracy of 99.36%, which is pretty goood for the model.


### Sliding Window Search
In this step, I used Sliding Window Search to scan images for the objects we are looking for, in this case: cars. This algorithm is implemented in 2-steps: 
1. Pyramid search or Scale search: We slide the windows across the image at different scales. This is good to detect objects in varying sizes, like cars driving on the road that are at different distances from me.  This method is applied using `get_sliding_windows` in  `process` of class ObjectDetector. 
This method in-turn calls method get_sliding_windows passing it a different scale (or image size) on each iteration. get_sliding_windows returns an array of window points (rectangles). 

Configuration settings for the sliding windows algorithm:
sliding_windows_config = {
    'xy_overlap' : (0.75, 0.8),
    'xy_window' : [64, 64],
    'y_start_stop' : [320, 680],
    'x_start_stop' : [760, 1260],
    'window_sizes' : [60, 100, 150],
    'heat_threshold' : 2
}


Example of output of the vehicle detection pipeline. It shows positve hits form the sliding windows' blob detection, and final output bounding boxes:


1. output of the sliding windows algorithm, as implemented in the find_objects method. 
This shows the sliding windows that had positive detections. 

2.An issue with HOG feature extraction is that you can get lots of False Positives. To deal with this, I built up a "heatmap" of our sliding window detections. 
This is done by creating a 2D array with the same dimensions of the image. 

3. Shows the heatmap. The brighter pixels indicate that it was within lots of sliding windows detections. Then I apply threshold mask to the heat and use scipy.ndimage.measurements.label to perform blob detection.
This segregates our found sliding windows into detected individual objects as shown in the final image above. 


### Apply pipeline in the video. 


### Discussion


2. 