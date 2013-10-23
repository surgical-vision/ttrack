import os,shutil

f = [f for f in os.listdir('.') if f.find('frame_') >= 0]
f = sorted(f,key=lambda x: int(filter(str.isdigit,x)))

for dir in f:
  
  print 'processing dir: ' + dir
  
  
  src = dir + '/step_init.png'
  dst = 'all_frames/frame_' + str(len(os.listdir('all_frames'))) + '.png'
  print 'copying file: ' + src + ' --> ' + dst +'\n'
  shutil.copy(src,dst)
  
  for im in os.listdir(dir):
  
    if im.find('step_init') >= 0: 
      continue
    if im == 'debug':
      continue
    
    src = dir + '/' + im
    dst = 'all_frames/frame_' + str(len(os.listdir('all_frames'))) + '.png'
    print 'copying file: ' + src + ' --> ' + dst + '\n'
    shutil.copy(src,dst)
