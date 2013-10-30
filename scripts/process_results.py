import sys
from argparse import ArgumentParser
import os

class Pose(object):
  
  def __init__(self,rotations=[],translations=[]):
    self.rotations = rotations
    self.translations = translations
  
  def tx(self):
    return self.translations[0]
  def ty(self):
    return self.translations[1]
  def tz(self):
    return self.translations[2]
  
  def __str__(self):
    if self.rotations == []
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
    elif self.translations == []
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
    if self.rotations == []
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
  

def isfloat(string):
  
  try:
    map(float,[string])
    return True
  except:
    return False
  
def process_lines(lines,delimiter,skip_line_number=False):

  for line in lines:
    if line[0] == "#":
      continue
    l = line.split(delimiter)
    l = [v.strip(delimiter).strip(" ").strip("\n") for v in l if isfloat(v)]
    if skip_line_number:
      l = l[1:-1]
    yield l
    
parser = ArgumentParser(description="Process results and compare to ground truth")
parser.add_argument("--estimates","-e",action="store",dest="estimates",
                    help="The file containing the estimates of the object pose.")
parser.add_argument("--ground-truth","-gt",action="store",dest="ground_truth",
                    help="The file containing the ground truth object pose.")
parser.add_argument("--delimiter","-d",action="store",dest="delimiter",default=",",
                    help="The delimiting character used to separate values on each line.")
parser.add_argument("--skip-line-number",action="store_true",dest="skip_line_number",default=False,
                    help="Flag to specify whether the first number on each line is a frame number (and to skip it).")
parser.add_argument("--no-rotations",action="store_true",dest="no_rotations",default=False,
                    help="Flag to specify that no rotations are present in the pose estimations.")
parser.add_argument("--no-translations",action="store_true",dest="no_translation",default=False,
                    help="Flag to specify that no translations are present in the pose estimations.")

                    
args = parser.parse_args()
    
try:
  est = open(args.estimates,"r")
  gt = open(args.ground_truth,"r")
except IOError as e:
  args.print_help()
  sys.exit(1)
  
estimates = process_lines(est.readlines(),args.delimiter,args.skip_line_number)
pose_estimates = []
for estimate in estimates:
  if args.no_rotations:
    pose_estimates.append( Pose(translations=estimate) )
  elif args.no_translations:
    pose_estimates.append( Pose(rotations=estimate) )
  else:
    pose_estimates.append( Pose(rotations=estimate[0:3],translations=estimate[3:-1]) )
  

