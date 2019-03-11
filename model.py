## Behavioral Cloning Project
## Model v6.4
## Save as drive.py
## We are importing data.zip where we have our driving.log.csv and IMG folder

import csv
import cv2
import numpy as np
import random

EPOCHS = 16
# BATCH_SIZE = 128
LR = 0.001

lines=[]
with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)


# # Split data on a multiple of BATCH SIZE
# from sklearn.model_selection import train_test_split
#
# train_samples, valid_samples = train_test_split(lines, test_size=0.2, random_state=12)


images = []
measurements = []
for line in lines:
    for i in range(3):
        source_path = line[i]
        tokens = source_path.split('/')
        filename = tokens[-1]
        local_path = "./data/IMG/" + filename
        image = cv2.imread(local_path)   # images array
        images.append(image)
    correction = 0.13    # add correction factor for situations when we are off to the right or to the left
    measurement  = float(line[3])		# measurements array (4th token)
    measurements.append(measurement)
    measurements.append(measurement + correction) # take the left image as center and +0.2 to the steering angle used.
    measurements.append(measurement - correction) # take the left image as center and -0.2 to the steering angle used.
    #print(tokens)
    #exit

# Augment the data
augmented_images = [ ]
augmented_measurements = [ ]
for image, measurement in  zip(images, measurements):
    augmented_images.append(image)		# add the image
    augmented_measurements.append(measurement)  # add the measurement
    # Flip only 50% of images and add them to the augmented dataset.
    if (random == 0):
        flipped_image = cv2.flip(image,1)     	# Create the flip images over the vertical axis (axis=1)
        flipped_measurement = measurement* -1.0    # Flip measurement by just change sign. It's a string so I need to cast it to float
        augmented_images.append(flipped_image)		# append the flipped image
        augmented_measurements.append(flipped_measurement)		# append the flopped measurement

X_train = np.array(augmented_images)
y_train = np.array(augmented_measurements)
#X_train = np.array(images)
#y_train = np.array(measurements)

# # Generator
# def generator(samples, batch_size):
#     num_samples = len(samples)
#     while 1:
#         random.shuffle(samples)
#         #remove_low_steering(samples)
#         for offset in range(0, num_samples, batch_size):
#             batch_samples = samples[offset:offset+batch_size]

#             images=[]
#             measurements=[]
#             for batch_sample in batch_samples:
#                 for i in range(3):
#                     source_path = line[i]
#                     tokens = source_path.split('/')
#                     filename = tokens[-1]
#                     local_path = "./data/IMG/" + filename
#                     image = cv2.imread(local_path)   # images array
#                     images.append(image)
#                 correction = 0.12
#                 measurement  = float(batch_sample[3])
#                 measurements.append(measurement)
#                 measurements.append(measurement + correction)
#                 measurements.append(measurement - correction)

#             #Augment the data
#             augmented_images = [ ]
#             augmented_measurements = [ ]
#             for image, measurement in  zip(images, measurements):
#                 augmented_images.append(image)      # add the image
#                 augmented_measurements.append(measurement)  # add the measurement
#                 # Flip only 50% of images and add them to the augmented dataset.
#                 if (random == 0):
#                     flipped_image = cv2.flip(image,1)       # Create the flip images over the vertical axis (axis=1)
#                     flipped_measurement = measurement* -1.0    # Flip measurement by just change sign. It's a string so I need to cast it to float
#                     augmented_images.append(flipped_image)      # append the flipped image
#                     augmented_measurements.append(flipped_measurement)      # append the flopped measurement

#             X_train = np.array(augmented_images)
#             y_train = np.array(augmented_measurements)
#             yield sklearn.utils.shuffle(X_train, y_train)
#
# # Prepare the train data using the generator function
# train_generator = generator(train_samples, batch_size = BATCH_SIZE)
# valid_generator = generator(valid_samples, batch_size = BATCH_SIZE)


# Network Architecture:
import keras
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout, SpatialDropout2D
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import Adam

model = Sequential()

model.add(Lambda(lambda x: x/255.0 - 0.5, input_shape =(160, 320, 3))) # normalization layer
model.add(Cropping2D(cropping=((70, 25),(0,0))))    # crop 70px from top, 25px from bottom
model.add(Convolution2D(24, 5, 5, subsample = (2,2), activation = 'relu'))  # 6 5x5 filters  + Relu Activation function
#model.add(SpatialDropout2D(0.2))
model.add(Convolution2D(36, 5, 5, subsample = (2,2), activation = 'relu'))  # 6 5x5 filters  + Relu Activation function
#model.add(SpatialDropout2D(0.2))
model.add(Convolution2D(48, 5, 5, subsample = (2,2), activation = 'relu'))
#model.add(SpatialDropout2D(0.2))
model.add(Convolution2D(64, 3, 3, activation = 'relu'))
#model.add(SpatialDropout2D(0.2))
model.add(Convolution2D(64, 3, 3, activation = 'relu'))
#model.add(SpatialDropout2D(0.2))

model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100)) #, activation = 'relu'))
model.add(Dense(50)) # , activation='relu'))
model.add(Dense(10)) #, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(1))


model.compile(optimizer=Adam(lr=LR), loss = 'mse')
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=EPOCHS)

# samples_per_epoch = BATCH_SIZE * np.ceil(len(train_samples)/BATCH_SIZE)
# nb_val_samples = len(valid_samples)
# model.fit_generator(train_generator, samples_per_epoch = samples_per_epoch,
#     validation_data = valid_generator, nb_val_samples = nb_val_samples,
#     nb_epoch = EPOCHS)

model.save('model_v6.4.h5')
import gc; gc.collect()
