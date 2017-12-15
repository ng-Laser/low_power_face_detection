# lots of code copied from http://dlib.net/dnn_face_recognition_ex.cpp.html
import sys
import os
import dlib
import glob
import numpy as np
from scipy import misc
from skimage import io
import serial           # for communicating with the board

# compares an image to a reference image to see if the 1 person in both images is the same
if len(sys.argv) != 2:
  print("give face to compare to as argument")
  exit()

predictor_path = "/home/noa/ee107/shape_predictor_5_face_landmarks.dat"
print("predictor path " + predictor_path)
face_rec_model_path = "/home/noa/ee107/dlib_face_recognition_resnet_model_v1.dat"
img_to_compare = sys.argv[1]

# Load all the models we need: a detector to find the faces, a shape predictor
# to find face landmarks so we can precisely localize the face, and finally the
# face recognition model.
detector = dlib.get_frontal_face_detector()
sp = dlib.shape_predictor(predictor_path)
facerec = dlib.face_recognition_model_v1(face_rec_model_path)

img1 = io.imread(img_to_compare)
img1 = misc.imresize(img1, .5)
dets1 = detector(img1, 1)
print("Number of faces detected: {}".format(len(dets1)))
if len(dets1) < 1:
   exit()
shape1 = sp(img1, dets1[0])  # address first face found in image
face_descriptor1 = facerec.compute_face_descriptor(img1, shape1)

reference_image = "/home/noa/ee107/noa_ref/noa_day2.png"
img2 = io.imread(reference_image)
img2 = misc.imresize(img2, .5)
dets2 = detector(img2, 1)
d2 = dets2[0]    # just taking the first face for now 
shape2 = sp(img2, d2) 
face_descriptor2 = facerec.compute_face_descriptor(img2, shape2)

# print("diff in descriptors " + face_descriptor1 - face_descriptor2)
fd1_np = np.asarray(face_descriptor1, dtype=float)
fd2_np = np.asarray(face_descriptor2, dtype=float)
diff = fd1_np - fd2_np
dist = np.sqrt(np.sum([x*x for x in diff]))

if dist < .6:
  # send acknowledgement to board that this is the right face
  ser = serial.Serial('/dev/ttyUSB0', timeout=1)
  ser.baudrate = 230400
  ser.write('a')
