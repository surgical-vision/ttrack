import sys,os,math
from argparse import ArgumentParser
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy


class Pose(object):
  
  def __init__(self,rotations=[],translations=[], articulations = [], use_quats=None):
  
    rotations = map(float,rotations)
    translations = map(float,translations)
    articulations = map(float,articulations)
    
    if use_quats:
      self.rotations = Pose.QuaternionToEuler(rotations)
    else:
      self.rotations = rotations

    self.translations = translations
    self.articulations = articulations


  @staticmethod
  def QuaternionToEuler(q):
    phi = math.atan2( 2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]) )
    theta = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
    psi = math.atan2( 2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]) )
    return (phi,theta,psi)
    
  @staticmethod
  def rotation_error(gt_val, est_val):
    return {
    "Roll error":math.fabs(gt_val[0] - est_val[0]),
    "Pitch error":math.fabs(gt_val[1] - est_val[1]),
    "Yaw error":math.fabs(gt_val[2] - est_val[2])
    }
  
  @staticmethod
  def translation_error(gt_val, est_val):
    return {
    "Tx error":math.fabs(gt_val[0] - est_val[0]),
    "Ty error":math.fabs(gt_val[1] - est_val[1]),
    "Tz error":math.fabs(gt_val[2] - est_val[2])
    }
    
  @staticmethod
  def articulation_error(gt_val, est_val):
    d = {}
    for i in range(len(gt_val)):
      d["DOF {0} error".format(i)] = math.fabs(gt_val[i]-est_val[i])
    return d
    
  def tx(self):
    return self.translations[0]
  def ty(self):
    return self.translations[1]
  def tz(self):
    return self.translations[2]
  
  def __str__(self):
    if self.rotations == []:
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
    elif self.translations == []:
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
    if self.rotations == []:
      return "[{0}, {1}, {2}]".format(self.tx(),self.ty(),self.tz())
  

def create_graph(xvals, yvals, xlabel, ylabel, savefile_name):

  ax = plt.plot(xvals, yvals)
  plt.ylabel(ylabel)
  plt.xlabel(xlabel)
  
  plt.savefig( savefile_name )

  
def write_error_vals_to_file(error_values, save_file):

  if len(error_values) == 0:
    return

  with open(save_file,"w") as f:
  
    for val in error_values[0]:
    
      values = [g[val] for g in error_values]
      mean = scipy.mean(values)
      std = scipy.std(values)
      
      f.write("{0}-mean:{1}\n".format(val,mean))
      f.write("{0}-std:{1}\n".format(val,std))
     

def create_rotation_errors_graph(error_values, save_dir):

  frames = range(len(error_values))
  
  roll_vals = [f["Roll error"] for f in error_values]
  create_graph( frames, roll_vals, "Frame no.", "Roll error (radians)", os.path.join(save_dir,"roll.png") )
  pitch_vals = [f["Pitch error"] for f in error_values]
  create_graph( frames, pitch_vals, "Frame no.", "Roll error (radians)", os.path.join(save_dir,"pitch.png") )
  yaw_vals = [f["Yaw error"] for f in error_values]
  create_graph( frames, yaw_vals, "Frame no.", "Yaw error (radians)", os.path.join(save_dir,"yaw.png") )
  
  write_error_vals_to_file(error_values, os.path.join(save_dir,"rotation_errors.txt")) 
  
def create_translation_errors_graph(error_values, save_dir):

  frames = range(len(error_values))
  
  tx_vals = [f["Tx error"] for f in error_values]
  create_graph( frames, tx_vals, "Frame no.", "Tx error (mm)", os.path.join(save_dir,"tx.png") )
  ty_vals = [f["Ty error"] for f in error_values]
  create_graph( frames, ty_vals, "Frame no.", "Ty error (mm)", os.path.join(save_dir,"ty.png") )
  tz_vals = [f["Tz error"] for f in error_values]
  create_graph( frames, tz_vals, "Frame no.", "Tz error (mm)", os.path.join(save_dir,"tz.png") )
  
  write_error_vals_to_file(error_values, os.path.join(save_dir,"translation_errors.txt")) 

def create_articulation_errors_graph(error_values, save_dir):
  
  frames = range(len(error_values))
  
  if len(frames) == 0:
    return
  
  for n,err_vals in enumerate(error_values[0]):
    vals = [f[err_vals] for f in error_values]
    create_graph( frames, vals, "Frame no.", "DOF {0} error (radians)".format(n), os.path.join(save_dir,"dof_{0}.png".format(n)) )
  
  write_error_vals_to_file(error_values, os.path.join(save_dir,"articulation_errors.txt")) 
  
  
def isfloat(string):
  
  try:
    map(float,[string])
    return True
  except:
    return False
  
def process_lines(lines,delimiter,skip_line_number=False):

  r = []
  
  for line in lines:
    if line[0] == "#":
      continue
    l = line.split(delimiter)
    l = [v.strip(delimiter).strip(" ").strip("\n") for v in l if isfloat(v)]
    if skip_line_number:
      l = l[1:-1]
    #yield l
    r.append(l)
  
  return r
    
    
if __name__ == '__main__':
  
  parser = ArgumentParser(description="Process results and compare to ground truth")
  parser.add_argument("--estimates","-e",action="store",
                      help="The file containing the estimates of the object pose.", required=True)
  parser.add_argument("--ground-truth","-gt",action="store",
                      help="The file containing the ground truth object pose.", required=True)
  parser.add_argument("--delimiter","-d",action="store", default=" ",
                      help="The delimiting character used to separate values on each line.")
  parser.add_argument("--skip-line-number",action="store_true",dest="skip_line_number",default=False,
                      help="Flag to specify whether the first number on each line is a frame number (and to skip it).")
  parser.add_argument("--no-rotation",action="store_true", default=False,
                      help="Flag to specify that no rotations are present in the pose estimations.")
  parser.add_argument("--no-translation",action="store_true", default=False,
                      help="Flag to specify that no translations are present in the pose estimations.")
  parser.add_argument("--no-articulation", action="store_true", default=False, 
                      help="Flag to specify that no articulations are present in the pose estimations.")
                      
  parser.add_argument("--use-ground-truth-quaternions",action="store_true",default=False,
                      help="Flag to specify that quaternions are used in the ground truth rather than euler angles")
  
  parser.add_argument("--use-estimates-quaternions",action="store_true",default=False,
                      help="Flag to specify that quaternions are used in the ground truth rather than euler angles")

  parser.add_argument("--use-quaternions",action="store_true",default=False,
                      help="Flag to specify that quaternions are used rather than euler angles")
                        
  
  args = parser.parse_args()
  
  if args.use_quaternions:
    args.use_ground_truth_quaternions = True
    args.use_estimates_quaternions = True
      
  try:
    est = open(args.estimates,"r")
    gt = open(args.ground_truth,"r")
  except IOError as e:
    args.print_help()
    sys.exit(1)
    
  estimates = process_lines(est.readlines(),args.delimiter,args.skip_line_number)
  ground_truths = process_lines(gt.readlines(), args.delimiter, 0)
  
  if not args.use_ground_truth_quaternions or not args.use_estimates_quaternions:
    print("Warning. Reading as Euler angles rather than quaternions\n")
  
  if len(estimates) != len(ground_truths):
    print("Warning. The number of estimates does not equal the number of ground truths\n")
  
  pose_estimates = []
  pose_ground_truths = []
  
  rotation_gt_idx = 6
  if args.use_ground_truth_quaternions:
    rotation_gt_idx = 7
  
  rotation_est_idx = 6
  if args.use_estimates_quaternions:
    rotation_est_idx = 7
  
  for estimate in estimates:
    if args.no_rotation:
      pose_estimates.append( Pose(translations=estimate, use_quats=args.use_estimates_quaternions) )
    elif args.no_translation:
      pose_estimates.append( Pose(rotations=estimate, use_quats=args.use_estimates_quaternions) )
    elif args.no_articulation:
      pose_estimates.append( Pose(rotations=estimate[3:rotation_est_idx],translations=estimate[0:3], use_quats=args.use_estimates_quaternions) )
    else:
      pose_estimates.append( Pose(rotations=estimate[3:rotation_est_idx],translations=estimate[0:3],articulations=estimate[rotation_est_idx:], use_quats=args.use_estimates_quaternions) )
    
  for ground_truth in ground_truths:
  
    if args.no_rotation:
      pose_ground_truths.append( Pose(translation=ground_truth, use_quats=args.use_ground_truth_quaternions) )
    elif args.no_translation:
      pose_ground_truths.append( Pose(rotations=ground_truth, use_quats=args.use_ground_truth_quaternions) )
    elif args.no_articulation:
      pose_ground_truths.append( Pose(rotations=ground_truth[3:rotation_est_idx],translations=ground_truth[0:3], use_quats=args.use_ground_truth_quaternions) )
    else:
      pose_ground_truths.append( Pose(rotations=ground_truth[3:rotation_gt_idx],translations=ground_truth[0:3], articulations=ground_truth[rotation_gt_idx:], use_quats=args.use_ground_truth_quaternions) )
  
  rotation_errors = []
  translation_errors = []
  articulation_errors = []
  
  for gt, est in zip(pose_ground_truths, pose_estimates):
    
    if not args.no_rotation:
      
      rotation_errors.append( Pose.rotation_error( gt.rotations, est.rotations ) )
      
    if not args.no_translation:

      translation_errors.append( Pose.translation_error( gt.translations , est.translations ) )
      
    if not args.no_articulation:

      articulation_errors.append( Pose.articulation_error( gt.articulations, est.articulations ) )
   

  estimate_dir = os.path.dirname(args.estimates)
  
  create_rotation_errors_graph(rotation_errors, estimate_dir)
  create_translation_errors_graph(translation_errors, estimate_dir)
  create_articulation_errors_graph(articulation_errors, estimate_dir)
  