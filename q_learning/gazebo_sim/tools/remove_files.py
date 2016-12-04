#!/usr/bin/env python
# -----------------------------------
# remove images starting from the entered index
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------
'''
Remove files in the folder in the given path
'''
import os
import sys
import numbers
import re

def rm_riles(path, index):
    if not os.path.exists(path):
        print 'path you entered did not exist.'
        return

    lst = os.listdir(path)
    for filename in lst:
        fileindex_list = re.findall(r'\d+', filename)
        fileindex = int(fileindex_list[0])
        if fileindex >= index:
            try:
                os.remove(os.path.join(path,filename))
            except OSError, e:
                print 'remove file error'
                print e

    print 'deleting files done...'



if __name__ == "__main__":
    dir = raw_input("please enter the full path of the directory: ")
    if dir == '\x03' or dir == '\x71':  # ctrl + c or 'q'
        sys.exit()


    index_inputed = False
    while not index_inputed:
        index = raw_input("please enter the starting index of the file that you wanna delete: ")
        if index == '\x03' or index == '\x71':  # ctrl + c or 'q'
            sys.exit()
        try:
            index = eval(index)
            if not isinstance(index, numbers.Integral):
                print "index is not an integer"
                print "Please enter it again..."
            else:
                print "you entered index = ", index
                index_inputed = True
        except NameError, e:
            print "You did not enter a number, please try again"

    rm_riles(dir, index)