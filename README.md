# Semantic Segmentation
### Overview

This project consists on building a *Fully Convolutional Network (FCN)* to perform Semantic Segmentation of image data. 

The goal is to predict the label of the pixels of a road in images from [Kitti Road dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php).

### Setup
##### GPU
`main.py` will check to make sure you are using GPU - if you don't have a GPU on your system, you can use AWS or another cloud computing platform.
##### Frameworks and Packages
Make sure you have the following is installed:
 - [Python 3](https://www.python.org/)
 - [TensorFlow](https://www.tensorflow.org/)
 - [NumPy](http://www.numpy.org/)
 - [SciPy](https://www.scipy.org/)

You may also need [Python Image Library (PIL)](https://pillow.readthedocs.io/) for SciPy's `imresize` function.

##### Dataset
Download the [Kitti Road dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php) from [here](http://www.cvlibs.net/download.php?file=data_road.zip).  Extract the dataset in the `data` folder.  This will create the folder `data_road` with all the training a test images.

The data provided comes in the form of a raw image taken from an onboard car camera, and its corresponding pixel-labeled image (road/ no-road).

Here are examples of both images: 

Raw Image          |  Labeled Image
:-----------------:|:-------------------------:
![alt text][image1]|  ![alt text][image2]


### Solution

The approach I followed was to build a FCN-8 network based on [Long et al. paper](https://people.eecs.berkeley.edu/~jonlong/long_shelhamer_fcn.pdf).  The key features of the FCN architecture:
- FCN consists in an Encodered network followed by a Decoder.
- The encoder is a VGG16 pre-trained in ImageNet. 
- Fully Connected layers of VGG16 are converted into Fully Convolutional layers using 1x1 Convolution. This process produces a class presence heat map in low resolution.
- The decoder upsamples the low resolution semantic feature maps using Transposed Convolutions. 
- At each stage, the upsampling is refined by adding features from coarser by higher resolution feature maps from lower layers in VGG16. 
- This is done thorugh skip connection after each convolution block. Essentially this adds more abstract, class-salient features from the previous pooled features. 

![alt text][image12]


Below are the steps of the implementation:

#### Step 1

Load the pre-trained VGG-16 network into TensorFlow. 

```
def load_vgg(sess, vgg_path):
    #  Assign required parts of model to variables
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    #  Use tf.saved_model.loader.load to load the model and weights
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    # Read the protobuf loaded by the model, and call layers 3, 4, 7 for training
    image_input = tf.get_default_graph().get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = tf.get_default_graph().get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = tf.get_default_graph().get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = tf.get_default_graph().get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7_out = tf.get_default_graph().get_tensor_by_name(vgg_layer7_out_tensor_name)
    
    return image_input, keep_prob, layer3_out, layer4_out, layer7_out
tests.test_load_vgg(load_vgg, tf) 
```

#### Step 2

Create the layers for the FCN, using the tensors from the VGG-16 network. Only the decoder part of 
the FCN had to be built. I applied a 1x1 convolution to the encoder layers, 
and then added the decoder layers to the network with upsampling followed by skip connection. 

```
def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):

    # Initialize weights with random normal distribution
    init = tf.random_normal_initializer(mean=0, stddev=1e-2)

    # Set boundary of regularizer
    reg = tf.contrib.layers.l2_regularizer(1e-3)

    # rename variables for simplicity
    #layer3_out, layer4_out, layer7_out = vgg_layer3_out, vgg_layer4_out, vgg_layer7_out

    # Apply 1x1 convolution to replace the fully connected layer
    fcn8 = tf.layers.conv2d(vgg_layer7_out, filters=num_classes, kernel_size=1, padding='SAME',
    	kernel_initializer = init,
    	kernel_regularizer= reg,
    	name='fcn8')

    # Uplsample fcn8 to match dimensions of layer 4 
    fcn9 = tf.layers.conv2d_transpose(fcn8, filters = vgg_layer4_out.get_shape().as_list()[-1], 
    	kernel_size=4, strides=(2,2), padding='SAME',
    	kernel_initializer = init,
    	kernel_regularizer= reg,
    	name='fcn9')

    # Add skip connection between fcn9 and layer 4
    fcn9_plus_4 = tf.add(fcn9, vgg_layer4_out, name='fcn9_plus_4')

    # Upsample fcn9 to match dimensions of layer 3
    fcn10 = tf.layers.conv2d_transpose(fcn9_plus_4, filters=vgg_layer3_out.get_shape().as_list()[-1],
    	kernel_size=4, strides=(2, 2), padding='SAME', 
    	kernel_initializer = init,
    	kernel_regularizer= reg,
    	name="fcn10")

    # Add skip connection between fcn10 and layer 3
    fcn10_plus_3 = tf.add(fcn10, vgg_layer3_out, name="fcn10_plus_3")

    # Upsample fcn10 to match dimensions with input image size
    fcn11 = tf.layers.conv2d_transpose(fcn10_plus_3, filters=num_classes,
    	kernel_size=16, strides=(8, 8), padding='SAME', 
    	kernel_initializer = init,
    	kernel_regularizer= reg,
    	name="fcn11")

    return fcn11

tests.test_layers(layers)

```

#### Step 3
Optimize the network, using *Cross Entropy* as loss function and *Adam* as the optimizer.

```
def optimize(nn_last_layer, correct_label, learning_rate, num_classes):

    # Reshape 4D tensors to 2D. row = pixel; column = class
    logits = tf.reshape(nn_last_layer, (-1, num_classes), name = 'fcn_logits')
    correct_label_reshaped = tf.reshape(correct_label, (-1, num_classes))

    # Calculate distance from actual labels using cross entropy 
    cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct_label_reshaped[:])

    # Calculate mean for total loss
    loss_op = tf.reduce_mean(cross_entropy, name = 'fcn_loss')

    # Adam optimization algorithm.
    train_op = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(loss_op, name="fcn_train_op")

    return logits, train_op, loss_op

tests.test_optimize(optimize)
```
#### Step 4
Define the train function, which takes as parameters number of epchs, batch size, loss function, optimizer operation, and placeholders for 
input images, label images and learning rate.

For training I set `keep_prob` to 0.5 and `learning_rate` to 0.0001

```
def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image, correct_label, keep_prob, learning_rate):
    prob = 0.5
    lr = 0.0001
    for e in range(epochs):

        log_losses =[]

        for img, label in get_batches_fn(batch_size):
            _, loss = sess.run([train_op, cross_entropy_loss], 
                               feed_dict={
                                        input_image:img, 
                                        correct_label:label, 
                                        keep_prob: prob, 
                                        learning_rate: lr})

            log_losses.append(float(loss))

            # Keep track of the progress by printing loss during training
            print("Epoch: {}\tLoss: {}".format(e+1, str(loss)))

        print("Average Loss = {:.3f}".format(np.mean(log_losses)))

tests.test_train_nn(train_nn)
```

#### Step 5
Finally, `run()` function trains the model. First, I build the net using `load_vgg`, `layers`, and `optimize` functions, and then 
we train the network using `train_nn` abd save the inference data.


### Results

To train the model, I used an Nvidia Tesla K80 GPU in Udacity Workspace. 

I ran it a handful of time, tweaking some parameters until I achieved optimal results. Below is an example of the inference results 
comparing the first iteration and the last one. On the first iteration, I used  a learning rate of 1e-2, epoch size of 20 and batch sie of 30. On the last iteration, I decreased the learning rate to 1e-4, increased the epoch size to 40 and decrease the batch size to 8. It can be appreciated how the segmentation in the last iteration contains more number of correctly predicted pixels of the 'road' class, which translates on higher performance as measured by Intersection over Union (IoU). 


       Iteration #1        |      Iteration # 5
:-------------------------:|:-------------------------:
![alt text][image3]        |  ![alt text][image4]

On average, the model decreases loss over time.
![alt text][image5]

Other results:

:-------------------------:|:-------------------------:
![alt text][image6]        |  ![alt text][image7]
:-------------------------:|:-------------------------:
![alt text][image8]        |  ![alt text][image9]
:-------------------------:|:-------------------------:
![alt text][image10]       |  ![alt text][image11]



[//]: # (Image References)

[image1]: ./images/raw_image.png "Raw Image"
[image2]: ./images/labeled_image.png "Labeled Image"
[image3]: ./images/um_000036-iter1.png "Iter 1"
[image4]: ./images/um_000036-iter5.png "Iter 5"
[image5]: ./images/loss_decrease.png "loss decrease"
[image6]: ./images/uu_000017.png "sample 6"
[image7]: ./images/uu_000021.png "sample 7"
[image8]: ./images/um_000046.png "sample 8"
[image9]: ./images/um_000063.png "sample 9"
[image10]: ./images/umm_000002.png "sample 10"
[image11]: ./images/umm_000061.png "sample 11"
[image12]: ./images/fcn-8.png "FCN-8"