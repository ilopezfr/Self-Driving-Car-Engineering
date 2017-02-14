
import time
import pickle
import csv
import operator
from itertools import groupby

import cv2
import numpy as np
import tensorflow as tf
from tensorflow.contrib.layers import flatten

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt


## Functions to visualize a dataset with images 

def get_images_and_counts(X, Y, count_data):
    """
    get the images, their labels, and count them. 
    credit to: https://gist.github.com/autojazari
    """
    images, labels, counts = [], [], []
    for label, count in count_data:
        images.append(X[Y.index(label)])
        labels.append(label)
        counts.append(count)
    return images, labels, counts

def group_classes(Y):
    return {key:len(list(group)) for key, group in groupby(Y)}

def group_classes_sorted(Y):
    data = group_classes(Y)
    return sorted(data.items(), key=lambda x:x[1], reverse=True)

def plot_axes(axes, images, labels, counts=None, is_count=False, pred_labels=None):    
    for i, ax in enumerate(axes.flat):
        # Plot image.
        ax.imshow(images[i], cmap='binary')
        # Show true and predicted classes.
        if list(counts):
            xlabel = "Count: {0}".format(counts[i])
            title = "Class: {0}".format(labels[i])
        else:
            xlabel = "True: {0}, Pred: {1}".format(cls_true[i], pred_labels[i])

        ax.set_xlabel(xlabel)
        ax.set_title(title)
        # Remove ticks from the plot.
        ax.set_xticks([])
        ax.set_yticks([])


def plot_signs(images, labels, counts=None, pred_labels=None):
    """Create figure but watch out for 43!"""
    count = len(images)
    fig, axes = plt.subplots(6, 7, figsize=(10, 10))
    fig.subplots_adjust(hspace=1, wspace=1)
    plot_axes(axes, images[:-1], labels[:-1], counts, is_count=True)

    
    
## Functions to Pre-process the images
def min_max_normalization(data, a=-0.5, b=0.5):
    data_max = np.max(data)
    data_min = np.min(data)
    return a + (b - a) * ((data - data_min) / (data_max - data_min))

def one_hot_encoding(y_train, y_test):
    from sklearn.preprocessing import LabelBinarizer
    labelBinarizer = LabelBinarizer()
    labelBinarizer.fit(y_train)

    y_train_one_hot = labelBinarizer.transform(y_train)
    y_test_one_hot = labelBinarizer.transform(y_test)
    return y_train_one_hot, y_test_one_hot

def center_normalization(X_train, X_test):
    x_train = X_train.astype(np.float32)
    x_train -= np.mean(x_train, axis=0)
    x_train /= np.std(x_train, axis=0)

    x_test = X_test.astype(np.float32)
    x_test -= np.mean(x_test, axis=0)
    x_test /= np.std(x_test, axis=0)
    return x_train, x_test

class DataSet:
    def __init__(self, X, y):
        self.X = X
        self.y = y

        self.pointer = 0
        self.dataset_length = len(y)

    def next_batch(self, size):
        next_indices = np.arange(self.pointer, self.pointer + size) % self.dataset_length
        self.pointer += size
        self.pointer = self.pointer % self.dataset_length

        return self.X[next_indices], self.y[next_indices]

    def length(self):
        return self.dataset_length

# Visualize Learning Curves
def plot_learning_curves(training_losses, training_accuracies, dev_losses, dev_accuracies):
    '''
    Plot the accuracty-loss curves resulted from the training. 
    credit to: https://github.com/upul/traffic-signs/blob/master/Traffic_Signs_Recognition.ipynb
    '''
    import seaborn as sbs;
    sbs.set()
    epochs = np.arange(len(training_losses))
    plt.subplot(2, 1, 1)
    plt.plot(epochs, training_losses, color='#dd1c77', linewidth=2.0, label='training')
    plt.plot(epochs, dev_losses, color='#c994c7', linewidth=2.0, label='dev')

    # plt.xlabel('epoch')
    plt.ylabel('loss')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(epochs, training_accuracies, color='#dd1c77', linewidth=2.0, label='training')
    plt.plot(epochs, dev_accuracies, color='#c994c7', linewidth=2.0, label='dev')

    plt.xlabel('epoch')
    plt.ylabel('accuracy')
    plt.legend()

    plt.savefig('learning_curves.jpg')
    plt.show()
    
    
# Functions to perform transformations to images 

def transform_image(img, ang_range, shear_range, trans_range):
    """
    This function transforms images to generate new images.
    The function takes in following arguments,
    1- Image
    2- ang_range: Range of angles for rotation
    3- shear_range: Range of values to apply affine transform to
    4- trans_range: Range of values to apply translations over. 
    
    A Random uniform distribution is used to generate different parameters for transformation
    
    Copied from confluence post
    https://carnd-udacity.atlassian.net/wiki/display/CAR/Project+2+%28unbalanced+data%29+Generating+additional+data+by+jittering+the+original+image
    """
    # Rotation
    ang_rot = np.random.uniform(ang_range)-ang_range/2
    rows,cols,ch = img.shape    
    Rot_M = cv2.getRotationMatrix2D((cols/2,rows/2),ang_rot,1)

    # Translation
    tr_x = trans_range*np.random.uniform()-trans_range/2
    tr_y = trans_range*np.random.uniform()-trans_range/2
    Trans_M = np.float32([[1,0,tr_x],[0,1,tr_y]])

    # Shear
    pts1 = np.float32([[5,5],[20,5],[5,20]])

    pt1 = 5+shear_range*np.random.uniform()-shear_range/2
    pt2 = 20+shear_range*np.random.uniform()-shear_range/2

    pts2 = np.float32([[pt1,5],[pt2,pt1],[5,pt2]])

    shear_M = cv2.getAffineTransform(pts1,pts2)
        
    img = cv2.warpAffine(img,Rot_M,(cols,rows))
    img = cv2.warpAffine(img,Trans_M,(cols,rows))
    img = cv2.warpAffine(img,shear_M,(cols,rows))
    
    return img


def display_augmented_images(image_dataset, augmented_data, n_rows):
    '''
    Simple utility function for displaying augmented images.
    credit to: https://github.com/upul/traffic-signs/blob/master/Traffic_Signs_Recognition.ipynb
    '''
    plt.figure(figsize=(5,7.5))
    selected_classes = np.random.randint(0, 44, size=n_rows)
    image_number = 1
    for row in selected_classes:
        x_selected = X_train[np.argmax(y_train, axis=1) == row]
        index = 0 # print first image of each image category
        
        plt.subplot(n_rows, 2, image_number)
        plt.imshow(x_selected[index, :, :, :]) 
        plt.axis('off')
        plt.title('class: {}'.format(row)) 
        image_number += 1
        
        aug_selected = augmented_data[np.argmax(y_train, axis=1) == row]
        
        plt.subplot(n_rows, 2, image_number)
        plt.imshow(aug_selected[index, :, :, :]) 
        plt.axis('off')
        plt.title('class: {}'.format(row))
        image_number += 1
        
    plt.suptitle('Original Image (First Column) and Augmented Image (Second Column)')
    plt.show()
    
    
    
# Process new images and display them:
def resize_image(image_file):
    image = plt.imread(NEW_IMAGES_FOLDER + image_file)
    return scipy.misc.imresize(image, (32, 32))

def display_new_images(imgs_data):
    index = 1
    plt.figure(figsize=(4,9))
    for img, name in imgs_data:
        plt.subplot(5, 1, index)
        plt.imshow(img)
        plt.axis('off')
        plt.title(name)
        index += 1
    plt.show() 


# Print results when predicting with new data 
def print_result(ground_truth, top_k_prob, top_k_indices):
    '''
    Print the predictions of the new images and show a bar chart with the softmax probabilities.   
    credit to: https://github.com/upul/traffic-signs/blob/master/Traffic_Signs_Recognition.ipynb
    '''
    class_names = pd.read_csv('./signnames.csv')['SignName'].values
    index = 0
    img_index = 0
    plt.figure(figsize=(14, 11))
    gs = gridspec.GridSpec(5, 2, width_ratios=[1, 0.45]) 
    for key in ground_truth:
        img, tag = resized_image_data[img_index]
        img_index += 1
        plt.subplot(gs[index])
        plt.imshow(img)
        plt.axis('off')
        plt.title(tag)
        index += 1

        plt.subplot(gs[index])
        plt.barh(np.arange(1, 6, 1), 
                 top_k_prob[key, :],
                 0.8, 
                 color='#dd1c77')
        plt.yticks(np.arange(1, 6, 1), class_names[top_k_indices[key, :]])
        index += 1
    plt.suptitle('Test Images and their Softmax Probabilities')
    plt.show()
    
    
    