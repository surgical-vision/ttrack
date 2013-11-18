import os
import cv2
import numpy as np
import sys

sys.path.append("c:/users/max/projects")

from cv_utils.recolor.recolor import ColorSpace

def get_size(image_paths,image_dir,mask_dir):

  size = 0

  for file in image_paths:
    im = cv2.imread(os.path.join(image_dir,file))
    mask = cv2.imread(os.path.join(mask_dir,file))

    if im.shape[0:2] != mask.shape[0:2]:
      raise Exception("Error, mask and image are not the same size for {0} and {1}".format(os.path.join(image_dir,file),os.path.join(mask_dir,file)))

    size = size + im.shape[0]*im.shape[1]

  return size


def get_paths(files,image_path,mask_path):

  for file in files:
    yield (os.path.join(image_path,file),os.path.join(mask_path,file))


if len(sys.argv) != 3:
  image_path = "c:/users/max/projects/phd/ttrack/scripts/in_vivo/training_images/"
  mask_path = "c:/users/max/projects/phd/ttrack/scripts/in_vivo/masks/"
  #print "Error, run as: train.py IMAGE_DIR MASK_DIR"
  #sys.exit(1)
else:  
  image_path = sys.argv[1]
  #image = cv2.imread(image_path)
  mask_path = sys.argv[2]
  #mask = cv2.imread(mask_path,0)

files = os.listdir(image_path)
size = get_size(files,image_path,mask_path)

training_data = np.zeros( shape=(size,4), dtype=np.float32 )
training_labels = np.zeros( shape=(size,1), dtype=np.int32 )

row_index = 0


for image_path,mask_path in get_paths(files,image_path,mask_path):

  c = ColorSpace(image_path)
  hue = c.get_hue()
  sat = c.get_sat()
  o2 = c.get_o2()
  o3 = c.get_o3()
  mask = cv2.imread(mask_path,0)

  for r in range(hue.shape[0]):
    for c in range(hue.shape[1]):
  
      training_data[row_index,:] = np.asarray( [np.float32(hue[r,c]),np.float32(sat[r,c]),np.float32(o2[r,c]),np.float32(o3[r,c])] )
    
      if mask[r,c] >= 127:#255:
        training_labels[row_index,0] = np.int32(1)
      else:#elif mask[r,c] == 0:
        training_labels[row_index,0] = np.int32(0)
      #else:
      #  raise Exception("Bad label: {0}\n".format( mask[r,c] ) )
    
      row_index = row_index + 1

 
if row_index != size:
  raise Exception("Error, training matrix not filled!\n")

print "Done loading images!\n"


rf = cv2.RTrees()
    
rf.train(training_data,cv2.CV_ROW_SAMPLE,training_labels, params = {"max_depth" : 10, 
                                                                   "min_sample_count" : 50, 
                                                                   "regression_accuracy": 0.0, 
                                                                   "use_surrogates" : False,
                                                                   "max_categories" : 10, 
                                                                   "priors" : np.asarray([0.5,0.5]),  
                                                                   "calc_var_importance" : True , 
                                                                   "nactive_vars" : 0, 
                                                                   "max_num_of_trees_in_the_forest" : 50, 
                                                                   "forest_accuracy" : 0.0001 , 
                                                                   "termcrit_type" : cv2.TERM_CRITERIA_MAX_ITER})

rf.save("./rf.xml")

print "Done training!\n"
