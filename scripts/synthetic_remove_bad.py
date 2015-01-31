import os
os.environ["PATH"] = "c:/users/max/projects/opencv/build32/install/x86/vc12/bin/" + ";" + os.environ["PATH"]

import cv2
import sys
import shutil

good_dir = "good"
bad_dir = "bad"

def check_intersection_with_side(image):

  rows,cols = image.shape

  top = False
  left = False
  bottom = False
  right = False
  
  num_intersections = 0
  for c in range(cols):
    if image[0,c] == 255:
      #num_intersections+=1
      top = True
      break
  for c in range(cols):
    if image[rows-1,c] == 255:
      bottom = True
      break
  for r in range(rows):
    if image[r,0] == 255:
      left = True
      break
  for r in range(rows):
    if image[r,cols-1] == 255:
      right = True
      break
  
  return top,left,bottom,right

  
def iterate_along_and_check(image,row_range,col_range, re_row_range, re_col_range):

  contig_length_at_edge = 0
  contig_length_inside_edge = 0
  
  for r in row_range:
    for c in col_range:
      if image[r,c] == 255:
        contig_length_at_edge += 1
      
  for r in re_row_range:
    for c in re_col_range:
      if image[r,c] == 255:
        contig_length_inside_edge += 1
  
  return contig_length_at_edge > contig_length_inside_edge
  
  
def check_intersection(im,top,left,bottom,right):

  rows,cols = im.shape
  
  if top and iterate_along_and_check(image,[0],range(cols),[5],range(cols)):
    return True
  elif bottom and iterate_along_and_check(image,[rows-1],range(cols),[rows-6],range(cols)):
    return True
  elif left and iterate_along_and_check(image,range(rows),[0],range(rows),[5]):
    return True
  elif right and iterate_along_and_check(image,range(rows),[cols-1],range(rows),[cols-6]):
    return True
  
  
  
def rmfile(f):

  im = cv2.imread(f,0)
  
  top,left,bottom,right = count_intersection_with_side(im)
  
  if top + left + bottom + right == 1 and check_intersection(im,top,left,bottom,right):
    shutil.copy(f,os.path.join(bad_dir,f))
  else:
    shutil.copy(f,os.path.join(good_dir,f))
  

if __name__ == '__main__':

  try:
    os.chdir("../debug/test")
    os.mkdir(good_dir)
    os.mkdir(bad_dir)
  except:
    print "change to scripts directory"
    sys.exit(1)
    
    
  im_files = [f for f in os.listdir(".") if os.path.splitext(f) == ".png"]
  
  map( rmfile,im_files )
  
  