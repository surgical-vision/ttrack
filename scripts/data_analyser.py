#!/usr/bin/python

from argparse import ArgumentParser
import os

if __name__ == '__main__':

    parser = ArgumentParser(description="Do statistical analysis on training data")
    parser.add_argument('--root-dir','-d',action='store',dest='root_dir',default='data',
                        help='The root directory where the training data is stored')

    
    args = parser.parse_args()

    try:
        data_sets = [args.root_dir+file_path for file_path in os.listdir(args.root_dir)]
    except OSError as e:
        print "Error reading directory: " + args.root_dir + "\n" + e.strerror
        exit(1)

        
    for data_set in data_sets:
        
        

