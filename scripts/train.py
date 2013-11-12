import os
import cv2
import numpy as np
import sys

sys.path.append("c:/users/max/projects")

from cv_utils.recolor.recolor import ColorSpace

if len(sys.argv) != 3:
  print "Error, run as: train.py IMAGE_PATH MASK_PATH"
  sys.exit(1)
  

image_path = sys.argv[1]
image = cv2.imread(image_path)
mask_path = sys.argv[2]
mask = cv2.imread(mask_path,0)

training_data = np.zeros( shape=(image.shape[0]*image.shape[1],4), dtype=np.float32 )
training_labels = np.zeros( shape=(image.shape[0]*image.shape[1],1), dtype=np.int32 )

c = ColorSpace(image_path)
hue = c.get_hue()
sat = c.get_sat()
o2 = c.get_o2()
o3 = c.get_o3()

for r in range(image.shape[0]):
  for c in range(image.shape[1]):
  
    training_data[r*image.shape[1] + c,:] = np.asarray( [np.float32(hue[r,c]),np.float32(sat[r,c]),np.float32(o2[r,c]),np.float32(o3[r,c])] )
    
    if mask[r,c] == 255:
      training_labels[r*image.shape[1] + c,0] = np.int32(1)
    elif mask[r,c] == 0:
      
      training_labels[r*image.shape[1] + c,0] = np.int32(0)
    else:
      raise Exception("Bad label: {0}\n".format( mask[r,c] ) )
    
rf = cv2.RTrees()

rf.train(training_data,cv2.CV_ROW_SAMPLE,training_labels)

rf.save("./rf.xml")

