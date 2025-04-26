#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：Freeway.py 
@File    ：fig7.py
@Author  ：Lu
@Date    ：2023/12/9 19:54 
'''

import matplotlib.pyplot as plt
import pandas as pd
# 全局设置字体和字体大小
plt.rcParams['font.family'] = 'serif'  # 使用衬线字体，如 'serif'、'sans-serif'
plt.rcParams['font.size'] = 18
# 读取数据，指定逗号为分隔符
without_platooning_data = pd.read_csv('output_data_speed_not_vp.txt', delimiter=',')
with_platooning_data = pd.read_csv('output_data_speed_vp.txt', delimiter=',')

# 提取时间、平均速度、最大速度和最小速度数据
time_without_platooning = without_platooning_data['Time']
speed_without_platooning = without_platooning_data['AverageSpeed']
max_speed_without_platooning = without_platooning_data['MaxSpeed']
min_speed_without_platooning = without_platooning_data['MinSpeed']

time_with_platooning = with_platooning_data['Time']
speed_with_platooning = with_platooning_data['AverageSpeed']
max_speed_with_platooning = with_platooning_data['MaxSpeed']
min_speed_with_platooning = with_platooning_data['MinSpeed']

# 绘制曲线对比图
plt.figure(figsize=(12, 8))

# 添加阴影区域
plt.fill_between(time_without_platooning, min_speed_without_platooning, max_speed_without_platooning,  alpha=0.3, label='Speed Range (Non-platooning)', color='orange')
plt.fill_between(time_with_platooning, min_speed_with_platooning, max_speed_with_platooning,  alpha=0.3, label='Speed Range (Platooning)', color='skyblue')
plt.axvline(x=30, color='r', linestyle='-.', label='Pass through the bottleneck')
plt.plot(time_without_platooning, speed_without_platooning, label='Non-platooning', lw=2.5, color='orange')
plt.plot(time_with_platooning, speed_with_platooning, label='Platooning', lw=2.5, color='skyblue')

plt.annotate('Start platooning', xy=(5.0, 26.58), xytext=(-70, 120), textcoords='offset points', arrowprops=dict(arrowstyle="->"), fontsize=14)

ax = plt.gca()
ax.spines['bottom'].set_linewidth(3)
ax.spines['left'].set_linewidth(3)
ax.spines['right'].set_linewidth(3)
ax.spines['top'].set_linewidth(3)

# 添加标题和标签
# plt.title('Average Speed vs Time')
plt.xlabel('Simulation time(s)',  size = 20)
plt.ylabel('Average Speed(m/s)',  size = 20)
plt.legend(loc="lower right", fontsize=14)  # 添加图例

# 显示图形
plt.show()
