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
        max_pull_force = filtered_df['pulling_force'].max()/1000
        if max_pull_force > 7:
            print('Ignored file: ' + file)
            continue
        max_pull_forces.append(max_pull_force)
        mean_gripper_current = filtered_df['gripper_current'].mean()/1000
        mean_gripper_currents.append(mean_gripper_current)
        results.append(['Flat' if 'flat' in dirname else 'Random', 'Position' if 'position' in dirname else 'Current' if 'current' in dirname else 'Tactile', filename, max_pull_force, mean_gripper_current])
        print(f'{file}: force: {max_pull_force}, current: {mean_gripper_current}')

    print(f'{dirname}: mean force: {statistics.mean(max_pull_forces)} N, mean current: {statistics.mean(mean_gripper_currents)} A')
results_df = pd.DataFrame(data=results, columns=["Scenario", "Policy", "Run", "Pulling Force [N]", "Current [A]"])
print(results_df)
seaborn.set_style("whitegrid")
seaborn.axes_style({'font.family': ['serif']})
plt.rcParams['font.family'] = 'serif'
plt.rcParams.update({'font.size': 16})
gfg = seaborn.relplot(results_df, x='Current [A]', y='Pulling Force [N]', hue='Policy', col="Scenario", aspect=1.6, facet_kws={'ylim': (0,4)})
plt.show()

# fabric_grasping/scripts/data_reader.py Experiments/flat_current_10 Experiments/flat_position_10 Experiments/flat_tactile_900_18_2 Experiments/random_current_10 Experiments/random_position_10 Experiments/random_tactile_900_18_2
