#! /usr/bin/env python3

import os
import sys
import statistics
import pandas as pd
import seaborn
import matplotlib.pyplot as plt

results = list()

for dirname in sys.argv[1:]:

    max_pull_forces = []
    mean_gripper_currents = []

    for filename in [fn for fn in os.listdir(dirname) if fn.endswith('.csv')]:
        file = os.path.join(dirname, filename)
        df = pd.read_csv(file, sep=', ', engine='python')
        filtered_df = df.loc[df['state'] == 'PULL']
        max_pull_force = filtered_df['pulling_force'].max()
        max_pull_forces.append(max_pull_force)
        mean_gripper_current = filtered_df['gripper_current'].mean()
        mean_gripper_currents.append(mean_gripper_current)
        results.append([dirname, filename, max_pull_force, mean_gripper_current])
        print(f'{file}: force: {max_pull_force}, current: {mean_gripper_current}')

    print(f'{dirname}: mean force: {statistics.mean(max_pull_forces)/1000} N, mean current: {statistics.mean(mean_gripper_currents)/1000} A')

results_df = pd.DataFrame(data=results, columns=["experiment", "run", "force", "current"])
print(results_df)
seaborn.scatterplot(results_df, x='current', y='force', hue='experiment')
plt.show()