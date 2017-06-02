#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 26 10:24:07 2017

@author: pach0
"""

import os
from fnmatch import fnmatch
import pandas as pd

root = '/home/pach0/Documents/autonomous_bicycle/code/'
pattern = "*.csv"
filenames = []
files = []

for path, subdirs, files in os.walk(root):
    for name in files:
        if fnmatch(name, pattern):
            filenames.append(os.path.join(path, name))

[files.append(pd.read_csv(f)) for f in filenames]


list_columns = []
list_columns.append('.header.stamp.secs')
list_columns.append('.header.stamp.nsecs')
list_columns.append('.orientation.x')
list_columns.append('.orientation.z')
list_columns.append('.orientation.p')

file_0 = pd.read_csv(filenames[0])
file_1 = pd.read_csv(filenames[1])

columns_valid = file_0.columns.values.tolist()
df_columns_valid = pd.DataFrame(columns_valid)
df_columns_all = pd.DataFrame(list_columns)

df_filtered = pd.merge(df_columns_valid, df_columns_all,  how='inner')

file_0.shape
file_0 = file_0.filter(items=list(df_filtered.values.flatten()))
file_0.shape
                       
result = pd.merge(file_0[df_filtered.values], file_1[df_filtered.values.tolist()], 
                  how='outer', indicator=True, suffixes=('_x', '_y'),
                  on=['.header.stamp.secs', '.header.stamp.nsecs'])









