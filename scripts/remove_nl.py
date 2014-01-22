import sys

try:
  infile = open(sys.argv[1],"r")
  outfile = open(sys.argv[2],"w")
except:
  print("Error, bad command line args!\n")
  
  
outfile.write( infile.read().replace("\\\n","") )