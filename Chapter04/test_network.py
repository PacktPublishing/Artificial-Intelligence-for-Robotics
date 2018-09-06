# Test Network Program
# part of the toy  detector
# uses LeNet to detect if toys are in the image
#
# Francis X. Govers 2018
#
# references:
# https://www.pyimagesearch.com/2017/12/18/keras-deep-learning-raspberry-pi/
#
# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import imutils
import cv2



# load the image
image = cv2.imread("toy2.jpg")
orig = image.copy()

# pre-process the image for classification
image = cv2.resize(image, (128, 128))
image = image.astype("float") / 255.0
image = img_to_array(image)
image = np.expand_dims(image, axis=0)

# load the trained convolutional neural network
print("[INFO] loading network...")
model = load_model("toy_not_toy.model")

# classify the input image
(nottoy, toy) = model.predict(image)[0]
print("toy = ",toy, " Not Toy = ",nottoy)
# build the label
label = "toy" if toy > nottoy else "Not toy"
probability = max(toy, nottoy)
label = "{}: {:.2f}%".format(label, probability * 100)

# draw the label on the image
output = imutils.resize(orig, width=400)
cv2.putText(output, label, (200, 25),  cv2.FONT_HERSHEY_SIMPLEX,
	0.7, (200, 255, 40), 2)

# show the output image
cv2.imshow("Output", output)
cv2.imwrite("toy_classify2.jpg", output)
cv2.waitKey(0)
