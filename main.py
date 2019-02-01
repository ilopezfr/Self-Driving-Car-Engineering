#!/usr/bin/env python3
import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests
import numpy as np


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """

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


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function

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


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """

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


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image, correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    print('Starting training... for {} epochs'.format(epochs))
    print()

    prob = 0.5
    lr = 0.0001
    for e in range(epochs):
        print("Epoch {} ...".format(e+1), end='')
        print()
        # Create empty array to hold temporary losses
        log_losses =[]
        #total_loss = 0
        #for i, (img, label) in enumerate(get_batches_fn(batch_size)):
        for img, label in get_batches_fn(batch_size):
            _, loss = sess.run([train_op, cross_entropy_loss], 
                               feed_dict={
                                        input_image:img, 
                                        correct_label:label, 
                                        keep_prob: prob, 
                                        learning_rate: lr})
            #total_loss += loss
            log_losses.append(float(loss))

            # Keep track of the progress by printing loss during training
            #print("Epoch: {}\tBatch: {}\tLoss: {}".format(e+1, i, total_loss))
            print("Epoch: {}\tLoss: {}".format(e+1, str(loss)))

        print("Average Loss = {:.3f}".format(np.mean(log_losses)))
        print()
    print('Training finished.')

tests.test_train_nn(train_nn)


def run():
    EPOCHS = 40
    BATCH_SIZE = 8
    NUM_CLASSES = 2
    IMAGE_SHAPE = (160, 576)  # KITTI dataset uses 160x576 images
    DATA_DIR = '/data'
    RUNS_DIR = './runs'
    tests.test_for_kitti_dataset(DATA_DIR)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(DATA_DIR)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(DATA_DIR, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(DATA_DIR, 'data_road/training'), IMAGE_SHAPE)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # TODO: Build NN using load_vgg, layers, and optimize function
        image_input, keep_prob, layer3, layer4, layer7 = load_vgg(sess, vgg_path)

        # The resulting architecture form adding a decorder on top of the given vgg model
        model_output = layers(layer3, layer4, layer7, NUM_CLASSES)
        correct_label = tf.placeholder(tf.float32, [None, None, None, NUM_CLASSES], name='correct_label')
        learning_rate = tf.placeholder(tf.float32, name='learning_rate')

        # Returns the output logits, training operation and cost operation
        # - logits: each row is a pixel, each column a class
        # - train_op: function used to get the right parameters to the model to correctly label the pixels
        # - cross_entropy_loss: function outputting the cost which we are minimizing, lower cost should yield higher accuracy
        logits, train_op, cross_entropy_loss = optimize(model_output, correct_label, learning_rate, NUM_CLASSES)

        print("Model build successful, starting training...")

        # Initialize all variables
        sess.run(tf.global_variables_initializer())
        sess.run(tf.local_variables_initializer())

        # TODO: Train NN using the train_nn function
        #saver = tf.train.Saver()
        train_nn(sess, EPOCHS, BATCH_SIZE, get_batches_fn,
        	train_op, cross_entropy_loss, image_input, correct_label, keep_prob, learning_rate)



        # Run the model with the test images and save each painted output image (roads painted green)
        helper.save_inference_samples(RUNS_DIR, DATA_DIR, sess, IMAGE_SHAPE, logits, keep_prob, image_input)


        # OPTIONAL: Apply the trained model to a video

        #saver.restore(sess, './runs/sem_seg_model.ckpt')


if __name__ == '__main__':
    run()
