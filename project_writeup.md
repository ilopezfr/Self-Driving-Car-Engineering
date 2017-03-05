#**Behavioral Cloning Project**

This is a short walkthrough of the approach used to solve the problem of Udacity's "Behavioral Cloning".

The goal of the project is to use the simulator to drive the car around a track, collect data and train it on neural network so that the car can eventually predict the angle to steer by itself, and complete a drive on the simulator without crashing.

Steps of this project:
- Data Collection using the simulator
- Preprocessing the data
- Build Neural Network
- Train and validate the model on the GPU
- Test the model on the autonomous model using the simulator.
- Iterate.

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

The final model was trained on AWS g2.2xlarge EC2 instance for 16 epochs. 

Using Udacity's provided simulator and my drive.py file, the car can be driven autonomously around the track by executing:
`ssh python drive.py model.h5 `

[//]: # (Image References)

[image1]: ./images/camera_images.png

## Data Collection
I followed an iterative approach, starting with the collecting minimum data--one lap driving, in this case--, building a simple network, testing it on the simulator, measuring the results and then iterating. 

Here's an example of the three different types of images captured by the car:
![alt text][image1]

This way I was able to visualy appreciate what were the situations in which my network wasn't performing well so that I could collect more training data that would teach the network how to behave properly on the situations where it previously failed. 

By simulating this type of situations, I collected what we call "recovery data". I ended up collecting over 10,000 camera frames (including left, center, and right images). The final training data is a combination of:
- 1 lap center lane driving
- 1 lap center lane driving clockwise around the track.
- Recovery from the left and right side of the road. Special cases:
	- Curve prior to bridge.
	- Driving accross bridge.
	- Every sharp curve.
	- Sharp curve where there's no road delimiter but dirt.


In addition, I augmented the data during the training phase, which is explained in the next section. 

## Preprocessing

Apart from the data collected, I used data augmentation to generate additional data:
- I applied a correction factor of 0.13 to both Left and Right images so that I could use them as Center images and train with them. This increased my dataset x3. 
- I also flipped the images and took the opposite sign of the steering measurement, which helped with the left turn bias of the circuit. 
- I split my dataset into 80% for training and 20% for validation/testing.
- I cropped the images 70px from the top and 25px from the bottom portions of the images, as they don't contain useful information.

## Model Architecture

I used a model based on [Nvidia's paper](http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf). To reduce overfitting, I have added several dropout layers to the network.

![Network](https://raw.githubusercontent.com/dmonn/behavioral-cloning/master/model-visualization.png?token=AGHDd-jb9QsCb1Rw-pTEwTTm7wknuCmyks5YhQ0LwA%3D%3D "Network Visualization")

The network consists of five convolutional layers, followed by three fully connected layers. It includes RELU layers to introduce nonlinearity. The images are cropped and the data is normalized using a Keras lambda layer. I have added Dropout Layers before and after the fully connected layers to prevent overfitting.

```
____________________
Layer (type)                     Output Shape          Param #     Connected to
====================================================================================================
lambda_1 (Lambda)                (None, 66, 200, 3)    0           lambda_input_1[0][0]
____________________________________________________________________________________________________
cropping2d_1 (Cropping2D)        (None, 66, 200, 3)    0           lambda_1 [0][0]
____________________________________________________________________________________________________
convolution2d_1 (Convolution2D)  (None, 33, 100, 24)   1824        cropping2d_1[0][0]
____________________________________________________________________________________________________
convolution2d_2 (Convolution2D)  (None, 17, 50, 36)    21636       convolution2d_1[0][0]
____________________________________________________________________________________________________
convolution2d_3 (Convolution2D)  (None, 7, 23, 48)     43248       sconvolution2d_2[0][0]
____________________________________________________________________________________________________
convolution2d_4 (Convolution2D)  (None, 5, 21, 64)     27712       convolution2d_3[0][0]
____________________________________________________________________________________________________
convolution2d_5 (Convolution2D)  (None, 3, 19, 64)     36928       convolution2d_4[0][0]
____________________________________________________________________________________________________
flatten_1 (Flatten)              (None, 3648)          0           convolution2d_5[0][0]
____________________________________________________________________________________________________
dropout_1 (Dropout)              (None, 3648)          0           flatten_1[0][0]
____________________________________________________________________________________________________
dense_1 (Dense)                  (None, 100)           364900      dropout_1[0][0]
____________________________________________________________________________________________________
dense_2 (Dense)                  (None, 50)            5050        dense_1[0][0]
____________________________________________________________________________________________________
dense_3 (Dense)                  (None, 10)            510         dense_2[0][0]
____________________________________________________________________________________________________
dropout_2 (Dropout)              (None, 10)            0           dense_3[0][0]
____________________________________________________________________________________________________
dense_4 (Dense)                  (None, 1)             11          dropout_2[0][0]
====================================================================================================
Total params: 501,819
Trainable params: 501,819
Non-trainable params: 0
____________________________________________________________________________________________________

```


## Training Strategy and Solution Design Approach:
**Step 1** 
I drove one lap on the simulator and tested on the data collected a simple Neural Network with just a Lambda layer that normalizes the images (from 0, 255 to 0, 1), a Flatten layer and a Dense layer, and test it on the simulator. This was just to understand how the process works.

During the training of the network, the validation loss was going up and down for every epoch, which may indicated that we were overfitting. I decreased the number of epochs from 10 to 5 and it seemed to fix that. However, the loss seem still a bit high. 

When testing it on the simulator, the car crashed after 10 seconds. This didn't surprise me, as I was using very little train data and a simple network.


**Step 2** 
I improved the architecture by replicating LeNet network:
- 1 Lambda Layer that normalizes the images
- 1 Conv Layer with 6 5x5 filters and a Relu Activation
- 1 MaxPolling layer
- 1 Conv Layer with 16 5x5 filters and a Relu Activation
- 1 MaxPolling layer
- 1 Flatten Layer
- 3 Dense Layers

I chose Adam optimizer, as is the to-go optimizer for these types of problems, and MSE as the loss function, as we are predicting a a numeric value.


I trained the model, which seemed to have a smaller loss and not overfitting. Then added the model in the simulator but it still crashed in a few seconds. I noticed that the car tended to steer to the left and eventually crash.
This was because the track is not clock-wise and the dataset I was using only showed the car pooling to the left.

**Step 3** 
In order to fix this issue, I applied some of the suggestions from the class:
- Augment the data by driving and recording the car going the other way around the track (clockwise).
- Augment the data by flipping the images.

After adding the code on the model to mirror the images, I was able to augment the size of the training data 2x. 

I trained and tested it again on the simulator and the car was able to drive a bit further for ~20 seconds after crashing. I noticed thought that when the car steered to either left or right, it didn't know how to come back.

**Step 4**
To fix this, I took a few approaches:
- Add recovery data: I generated more train data by recording myself driving close to the side of the road and quickly turning back on to the middle of the road. I collected some data using this strategy while driving on particular parts of the track like: entering and exiting a sharp curve, and crossing the bridge.
- Leverage Left and Right images. I took the images collected from the left and right cameras, and applied a correction factor to the measurements so that I could use them as center images. I tested different values for the correction factor, from 0.2 to 0.1. I got best results with 0.12.
- Cropping the images: removed 70px from the top and 25px from the bottom portions of the images, as they don't contain useful information. This way, the model could train faster and focus on the portion of the image that is relevant for predicting a steering angle. To do so, I added Cropping2D Layer to the network. Adding this to the network instead of outside the model is more efficient as it takes advantage of the parallelization on the GPU. 

**Step 5**
Now it was time to improved the network to make it more robust.
I took the network architecture used on NVIDIA's paper and adopted to my data.

At this point, the model improved noticeably and was able to drive autonomously an entire lap without crashing. On the train phase, both the train loss and validation, although the gap between both had increased. Clearly a sign of overfitting. To confirm this on the simulator, driving on autonomous mode the car was still zigzagging a little bit n some areas where it was supposed to drive straight.

**Step 6**
To prevent the overfitting, I tested 2 approaches:
- I tuned my Adam optimizer with a small learning rate 0.001. Using a small learning rate allows me to get a generalized result while increasing the number of epochs, from 5 to 20.
- I added Dropout layers to my network. I tested both SpatialDropout and Dropout and different configurations of the layers. 

I considered using a Generator, as it was suggested. But after better selecting a useful training data and performing the appropriate transformation for better model generalization, the network didn't take too long to train on the AWS EC2 so I decided it wasn't necessary at this point. Given more time to build a network that could drive Circuit 2, I'd definitely use a Generator. 

I ran a few iterations adding and removing dropout layers and tuning the values for the parameters: number of epochs, learning rate and the fraction of the input units to drop (p). The values chosen in the drive.py file are those that performed the best results.