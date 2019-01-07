import cv2
import glob
from keras.models import Sequential
from keras.layers import Conv2D, Flatten, Dense, MaxPooling2D, Dropout, Activation
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers

###############################################################################

X_train = []
x_label = []

###############################################################################

import numpy as np

###############################################################################

def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

###############################################################################

print("Creating Dataset...")

for img_class, directory in enumerate(['dataset/red', 'dataset/yellow', 'dataset/green','dataset/none']):
    for i, file_name in enumerate(glob.glob("{}/*.jpg".format(directory))):
        file = cv2.imread(file_name)

        # file = cv2.cvtColor(file, cv2.COLOR_BGR2RGB);
        resized = cv2.resize(file, (224, 224))
        # cv2.imshow("Original image", resized)
        # cv2.imshow("Rotated image", rotateImage(resized,45))
        rot_image = rotateImage(resized, 45)
        flip_image = resized[::-1]
        # cv2.imshow("flipped image", resized[::-1])
        # cv2.waitKey()

        X_train.append(resized / 255.)
        X_train.append(rot_image / 255.)
        X_train.append(flip_image / 255.)
        x_label.append(img_class)
        x_label.append(img_class)
        x_label.append(img_class)

X_train = np.array(X_train)
x_label = np.array(x_label)


categorical_labels = to_categorical(x_label)
num_classes = 4
model = Sequential()
model.add(Conv2D(32, (3, 3), input_shape=(224, 224, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
model.add(MaxPooling2D(2,2))
Dropout(0.8)
model.add(Conv2D(32, (3, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
model.add(MaxPooling2D(2,2))
Dropout(0.8)
model.add(Flatten())
model.add(Dense(8, activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
model.add(Dense(num_classes, activation='softmax'))
loss = losses.categorical_crossentropy
optimizer = optimizers.Adam()
model.compile(loss=loss, optimizer=optimizer, metrics=['accuracy'])
model.summary()
model.fit(X_train, categorical_labels, batch_size=32, epochs=50, verbose=True, validation_split=0.1, shuffle=True)
score = model.evaluate(X_train, categorical_labels, verbose=True)
print(score)
model.save('new_tl_classifier_sim.h5')
print("Model Saved.")

###############################################################################