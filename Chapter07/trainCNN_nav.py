# -*- coding: utf-8 -*-
###
#CNN based robot navigation – TRAINING program
#@author: Francis Govers
#######

# import the necessary packages
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam
from sklearn.model_selection import train_test_split
from keras.preprocessing.image import img_to_array
from keras.utils import to_categorical
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
from keras.models import Sequential
from keras.layers.convolutional import Conv2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dense
from keras import backend as K

class ConvNet():
@staticmethod
	def create(width, height, depth, classes):
		# initialize the network
		network = Sequential()
		inputShape = (height, width, depth)
		# first set of CONV => RELU => POOL layers
		network.add(Conv2D(50, (10, 10), padding="same",
		input_shape=inputShape))
		network.add(Activation("relu"))
		network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
		# second set of CONV => RELU => POOL layers
		network.add(Conv2D(50, (5, 5), padding="same"))
		network.add(Activation("relu"))
		network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
		# third set of CONV => RELU => POOL layers
		network.add(Conv2D(50, (5, 5), padding="same"))
		network.add(Activation("relu"))
		network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
		# Fully connected ReLU layers
		network.add(Flatten())
		network.add(Dense(500))
		network.add(Activation("relu"))
		network.add(Dense(500))
		network.add(Activation("relu"))
		# softmax classifier
		network.add(Dense(classes))
		network.add(Activation("softmax"))
		# return the constructed network architecture
	return network

EPOCHS = 25
LEARN_RATE = 1e-3
BATCH = 32 # batch size - modify if you run out of memory

print ("Loading Images")
images=[]
labels=[]
#location of your images
imgPath = "c:\users\fxgovers\documents\book\chapter7\train\"
imageDirs=["left","right","center"]
for imgDir in imageDirs:
    fullPath = imgPath + imgDir
    # find all the images in this directory
    allFileNames = os.listdir(fullPath)
    ifiles=[]
    label = imgDirs.index(imgDir) # use the integer version of the label
    # 0= left, 1 = right, 2 = center
for fname in allFileNames:
    if ".jpg" in fname:
    ifiles.append(fname)

# process all of the images
for ifname in ifiles:
# load the image, pre-process it, and store it in the data list
    image = cv2.imread(ifname)
    # let's get the image to a known size regardless of what was collected
    image = cv2.resize(image, (800, 600))
    halfImage = 800*300 # half the pixels
    # cut the image in half -we take the top half
    image = image[0:halfimage]
    #size the image to what we want to put into the neural network
    image=cv2.resize(image,(224,224))
    # convert to grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #equalize the image to use the full range from 0 to 255
    # this gets rid of a lot of illumination variation
    image = cv2.equalizeHist(image)

    # gaussian blur the image to remove high freqency noise
    # we use a 5x kernel
    image = cv2.GaussianBlur(img,(5,5),0)

    # convert to a numpy array
    image = img_to_array(image)
    # normalize the data to be from 0 to 1
    image2 = np.array(image, dtype="float") / 255.0
    images=images.append(image)
    labels.append(label)
    labels = np.array(labels) # convert to array

# split data into testing data and training data 80/20
(trainData, testData, trainLabel, testLabel) = train_test_split(data,
labels, test_size=0.20, random_state=42)  
We have to convert the labels to be a tensor, which is just a particular data format.
# convert the labels from integers to vectors
trainLabel = to_categorical(trainLabel, num_classes=3)
testLabel = to_categorical(testLabel, num_classes=3)

# initialize the artificial neural network
print("compiling CNN...")
cnn = ConvNet.build(width=224, height=224, depth=1, classes=3)
opt = Adam(lr=LEARN_RATE, decay=LEARN_RATE / EPOCHS)
model.compile(loss="categorical_crossentropy", optimizer=opt,
metrics=["accuracy"])

# train the network
print("Training network. This will take a while")
trainedNetwork = model.fit_generator(aug.flow(trainImage, trainLabel,
batch_size=BATCH),
validation_data=(testImage, testLable), steps_per_epoch=len(trainImage) //
BATCH,
epochs=EPOCHS, verbose=1)
# save the model to disk
print("Writing network to disk")
cnn.save("nav_model")

#cnn.save(“toybox_model”)