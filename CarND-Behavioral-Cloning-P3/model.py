import pandas as pd
import numpy as np
import cv2
import os
import csv
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Sequential 
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint
from keras.layers import Lambda, Conv2D, MaxPooling2D, Dropout, Dense, Flatten

PATH = '/opt/carnd_p3/data/'

def to_rgb(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def flip(image):
    # flip on y axis or horizontal flip
    return cv2.flip(image, 1)

def crop_image(image):
    return image[60:130, :]

def resize_image(image, shape=(160, 70)):
    return cv2.resize(image, shape)

def processed_image(image):
    return resize_image(crop_image(image))

samples = []

def get_data():
    with open(PATH+'driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for line in reader:
            samples.append(line)

    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    return train_samples, validation_samples

def augument_data(batch_sample):
    steering_angle = np.float32(batch_sample[3])
    images = []
    steering_angles = []
    for img_path_idx in range(3):
        img_name = batch_sample[img_path_idx].split('/')[-1]
        img = cv2.imread(PATH+'/IMG/'+img_name)
        rgb = to_rgb(img)
        resized = processed_image(rgb)
        images.append(resized)

        if img_path_idx == 1:
            steering_angles.append(steering_angle+0.2)
        elif img_path_idx == 2:
            steering_angles.append(steering_angle-0.2)
        else:
            steering_angles.append(steering_angle)

        if img_path_idx == 0:
            center_flipped = flip(resized)
            images.append(center_flipped)
            steering_angles.append(-steering_angle)

    return images, steering_angles 

def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                # name = './IMG/'+batch_sample[0].split('/')[-1]
                # center_image = cv2.imread(name)
                # center_angle = float(batch_sample[3])
                aug_images, aug_angles = augument_data(batch_sample)
                images.extend(aug_images)
                angles.extend(aug_angles)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield shuffle(X_train, y_train)



def model():
    #NVIDIA Model:
    #5x5 Conv, filter 24, 2x2 stride, ELU
    #5x5 Conv, filter 36, 2x2 stride, ELU
    #5x5 Conv, filter 48, 2x2 stride, ELU
    #3x3 Conv, filter 64, 1x1 stride, ELU
    #3x3 Conv, filter 64, 1x1 stride, ELU
    #0.5 Dropout to reduce overfitting
    #Funnel down with Dense of 100, 50, 10 and 1 neurons, ELU
    #Last dense layer is the steering angle prediction

    model = Sequential()
    model.add(Lambda(lambda x:  (x / 127.5) - 1.0, input_shape=(70, 160, 3)))
    model.add(Conv2D(filters=24, kernel_size=5, strides=(2, 2), activation='elu'))
    model.add(Conv2D(filters=36, kernel_size=5, strides=(2, 2), activation='elu'))
    model.add(Conv2D(filters=48, kernel_size=5, strides=(2, 2), activation='elu'))
    model.add(Conv2D(filters=64, kernel_size=3, strides=(1, 1), activation='elu'))
    model.add(Conv2D(filters=64, kernel_size=3, strides=(1, 1), activation='elu'))
    model.add(Dropout(0.5))
    model.add(Flatten())
    model.add(Dense(100, activation='elu'))
    model.add(Dense(50, activation='elu'))
    model.add(Dense(10, activation='elu'))
    model.add(Dense(1))
    model.summary()
    return model

def train_model(model, train_samples, validation_samples, train_generator, validation_generator):
    #checkpoint = ModelCheckpoint('model-{epoch:03d}.h5', monitor='val_loss', verbose=0, save_best_only='true', mode='auto')
    model.compile(loss='mean_squared_error', optimizer=Adam(lr=0.001))
    #model.fit_generator(train_generator, samples_per_epoch=len(train_samples), validation_data=validation_generator, nb_val_samples=len(validation_samples), epochs=5)
    model.fit_generator(train_generator, steps_per_epoch= len(train_samples),
validation_data=validation_generator, validation_steps=len(validation_samples), epochs=5, verbose = 1)
    model.save('model.h5')

def main():
    train_samples, validation_samples = get_data()
    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=128)
    validation_generator = generator(validation_samples, batch_size=128)
    mdl = model()
    train_model(mdl, train_samples, validation_samples, train_generator, validation_generator)

if __name__ == '__main__':
    main()
