#!/usr/bin/env python
# -----------------------------------
# merge image folders
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------
'''
merge files in each subfolder in the given root path

e.g.
If you have the following folder tree:
--data
-----dataset1
---------back
---------car_pos
---------front
---------left
---------merged
---------right
-----dataset2
---------back
---------car_pos
---------front
---------left
---------merged
---------right
-----dataset2
---------back
---------car_pos
---------front
---------left
---------merged
---------right
And you want to merge these three datasets into:
--data
-----combined
---------back
---------car_pos
---------front
---------left
---------merged
---------right

Then just enter the directory of 'data', the program will automatically
generate a new folder called 'combined' in the data folder

'''
import os
import sys
import numbers
import re
import shutil

def get_max_index(path):
    files_lst = os.listdir(path)
    max_index = -1
    for filename in files_lst:
        fileindex_list = re.findall(r'\d+', filename)
        if not fileindex_list:
            continue
        fileindex = int(fileindex_list[0])
        if fileindex >= max_index:
            max_index = fileindex
    return max_index

def merge_files(parent_path):
    if not os.path.exists(parent_path):
        print 'path you entered did not exist.'
        return

    foldername_list = []

    dest_dir = os.path.join(parent_path, 'combined')
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    folder_lst = os.listdir(parent_path)

    for foldername in folder_lst:
        if foldername == 'combined':
            continue
        dest_count = 0
        folder_dir = os.path.join(parent_path, foldername)
        if not foldername_list:
            foldername_list = os.listdir(folder_dir)
            for fld_name in foldername_list:
                if not os.path.exists(os.path.join(dest_dir, fld_name)):
                    os.makedirs(os.path.join(dest_dir, fld_name))
        else:
            dest_count = get_max_index(os.path.join(dest_dir, foldername_list[0])) + 1

        src_count = get_max_index(os.path.join(folder_dir, foldername_list[0])) + 1


        for i in range(src_count):
            for fld_name in foldername_list:
                src_folder_dir = os.path.join(parent_path, foldername, fld_name)
                dest_folder_dir = os.path.join(dest_dir, fld_name)

                files_lst = os.listdir(src_folder_dir)
                first_filename = files_lst[0]
                fileindex_list = re.findall(r'\d+', files_lst[0])
                index_start = first_filename.index(fileindex_list[0])

                file_prefix = first_filename[:index_start]
                file_suffix = first_filename[index_start + len(fileindex_list[0]):]

                dest_file_name = file_prefix + str('{:06d}'.format(dest_count + i)) + file_suffix
                src_file_name = file_prefix + str('{:06d}'.format(i)) + file_suffix
                if not os.path.exists(os.path.join(src_folder_dir, src_file_name)):
                    continue
                shutil.copy2(os.path.join(src_folder_dir, src_file_name), os.path.join(dest_folder_dir, dest_file_name))

    print 'merging files done...'



if __name__ == "__main__":
    dir = raw_input("please enter the full path of the parent directory: ")
    if dir == '\x03' or dir == '\x71':  # ctrl + c or 'q'
        sys.exit()

    merge_files(dir)