#Import all necessary packages
import numpy
import keras
import csv
import cv2
from sklearn.model_selection import train_test_split
from keras.models import *
from keras.layers import *
from keras.optimizers import Adam
import matplotlib.pyplot as pyplot
import os

#Stop warnings related to tensorflow build from showing up
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

IMG_WIDTH = 66
IMG_HEIGHT = 200
IMG_CHANNELS = 3

imageArray = [] #Array that holds the images from the csv file
angleArray = [] #Array that holds the angle values from the csv file

setNum = 1 #Hold the number of the datasets
while(os.path.exists("datasets/dataset%i" % setNum)): #While there are still datasets left
    print("Getting data from dataset %i" % setNum) #Inform the user which dataset is being accessed
    firstLine = 0 #Used to ignore the first line of the csv file which contains the labels
    #Open the csv file
    with open('./datasets/dataset%i/data.csv' % setNum,'rt') as file:
        reader = csv.reader(file) #Create a reader to read the csv file line by line
        for line in reader: #For each line in the csv fileine == 1): #If it's not the first line
            if (firstLine == 0): #If on the first line
                firstLine = 1 #Set the flag to 1
            else:
                imagePath = "./datasets/dataset%i/png" % setNum + line[0].split('png')[1] + 'png' #Get the local path to the image
                image = pyplot.imread(imagePath) #Read the image
                image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV) #Change the image colors
                image = cv2.resize(image,(IMG_HEIGHT, IMG_WIDTH)) #Resize the image
                imageArray.append(image) #Add the image to the image array
                angleArray.append(float(line[1])) #Add the steering angle to the angle array
    setNum+=1

print("") #Create a gap between the data collection and training
	
assert (len(imageArray) == len(angleArray)) #Make sure that the number of images and angles is the same

#Change the image and angle arrays to numpy arrays so that the reshape function can be used
imageArray = numpy.array(imageArray) 
angleArray = numpy.array(angleArray)

#Split the image and angle arrays so that they can be used to train and validate the model
trainFirstHalf, validateFirstHalf = train_test_split(imageArray)
trainSecondHalf, validateSecondHalf = train_test_split(angleArray)

#Define the model
model = Sequential()	

#Add a normalization layer
model.add(BatchNormalization(input_shape=(IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS)))

#Add the convolutional layers
model.add(Conv2D(24,(5,5),strides=(2,2),activation='relu'))
model.add(Conv2D(36,(5,5),strides=(2,2),activation='relu'))
model.add(Conv2D(48,(5,5),strides=(2,2),activation='relu'))
model.add(Conv2D(64,(3,3),strides=(1,1),activation='relu'))
model.add(Conv2D(64,(3,3),strides=(1,1),activation='relu'))

#Flatten the convolutional layers
model.add(Flatten())

#Add all of the Dense layers
model.add(Dense(1164, activation='relu'))
model.add(Dense(100, activation='relu'))
model.add(Dense(50, activation='relu'))
model.add(Dense(10, activation='relu'))
model.add(Dense(1))

#Compile the model and fit it to the generated data set(s)
model.compile(loss='mse',optimizer=Adam())
history = model.fit(trainFirstHalf, trainSecondHalf, batch_size=128, epochs=1,validation_data=(validateFirstHalf, validateSecondHalf))

#Save the model as a json file and the weights as a h5 file
jsonFile = model.to_json()
with open('./model.json', "w") as jsonLoc:
	jsonLoc.write(jsonFile)
model.save_weights('./model.h5')