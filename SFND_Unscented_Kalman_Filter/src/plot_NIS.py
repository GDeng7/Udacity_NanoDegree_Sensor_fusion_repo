import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# plot NIS for radar
data_r = pd.read_csv('./build/NIS_RADAR.txt', sep=",")
data_r.head()
plt.figure(figsize=(10,6))
plt.plot(data_r['Radar NIS'], label='NIS Radar Data')
plt.plot([0,len(data_r['Radar NIS'])],[7.815,7.815],'r--',lw=2, label='Chi square = 7.815 for 3 DOF')
plt.xlabel('x')
plt.ylabel('y')
plt.title('NIS Radar vs Steps')
plt.legend()
plt.show()


# plot NIS for lidar
data_l = pd.read_csv('./build/NIS_LIDAR.txt', sep=",")
data_l.head()
plt.figure(figsize=(10,6))
plt.plot(data_l['Lidar NIS'], label='NIS Lidar Data')
plt.plot([0,len(data_l['Lidar NIS'])],[5.991,5.991],'r--',lw=2, label='Chi square = 5.991 for 2 DOF')
plt.xlabel('x')
plt.ylabel('y')
plt.title('NIS Lidar vs Steps')
plt.legend()
plt.show()
