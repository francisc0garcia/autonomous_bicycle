#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import pandas as pd
import glob, os

root = '/home/pach0/Documents/autonomous_bicycle/code/src/autonomous_bicycle/bags/csv/test_velocity_5/'
pattern = "*.csv"
text_common = 'test_velocity_5-bicycle-'

# get CSV files
files = glob.glob(root + pattern)

# define desired columns
list_columns = []
list_columns.append('time')

# IMU Data
list_columns.append('.orientation.x')
list_columns.append('.orientation.y')
list_columns.append('.orientation.z')
list_columns.append('.orientation.w')

list_columns.append('.angular_velocity.x')
list_columns.append('.angular_velocity.y')
list_columns.append('.angular_velocity.z')

list_columns.append('.linear_acceleration.x')
list_columns.append('.linear_acceleration.y')
list_columns.append('.linear_acceleration.z')

# GPS odometry Data
list_columns.append('.pose.pose.position.x')
list_columns.append('.pose.pose.position.y')
list_columns.append('.pose.pose.position.z')

# Real odometry Data
list_columns.append('.twist.twist.linear.x')

# Velocity GPS odometry Data
list_columns.append('.vector.x')
list_columns.append('.vector.y')
list_columns.append('.vector.z')

df_desired_columns = pd.DataFrame(list_columns)

# read all files
dfs = [pd.read_csv(fp, parse_dates=['time'])
           .drop_duplicates(subset=['time'])
           .set_index(['time'],
                      drop=False, verify_integrity=True) for fp in files]

#replace column name
for i in range(len(dfs)):
    columns_file = dfs[i].columns.values.tolist()
    df_filtered = pd.merge(pd.DataFrame(columns_file), df_desired_columns, how='inner')
    dfs[i] = dfs[i].filter(items=list(df_filtered.values.flatten()))

    names = dfs[i].columns.tolist()
    name_column_base = files[i].replace(root, '').replace(text_common, '').replace('.csv', '')

    print("{{" + str(names) + '}}')

    for column in names:
        if column != 'time':
            names[names.index(column)] = name_column_base + column
            dfs[i].columns = names

base_file = dfs[0]

for i in range(1, len(dfs)):
    # merge datasets using outer join
    base_file = pd.merge(base_file, dfs[i], how='outer', indicator=False, on=['time'])

print(base_file.shape)
#base_file.to_csv(root + 'result.csv')