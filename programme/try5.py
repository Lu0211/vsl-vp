import math

import numpy as np
import matplotlib.pyplot as plt

# 全局设置字体和字体大小
plt.rcParams['font.family'] = 'serif'  # 使用衬线字体，如 'serif'、'sans-serif'
plt.rcParams['font.size'] = 14
# 定义参数
vf = 120  # 自由流速度，单位 km/h
rho_max = 200  # 最大密度，单位 veh/km

# 新的参数
vf2 = 120 * 2 / 3
rho_max2 = 200

vf3 = 130
rho_max3 = 220

vf4 = 130 * 2 / 3
rho_max4 = 220

# 定义密度范围
rho = np.linspace(0, rho_max, 400)
rho_prime = np.linspace(0, rho_max2, 400)
rho3 = np.linspace(0, rho_max3, 400)
rho4 = np.linspace(0, rho_max4, 400)

# 计算流量
q = rho * vf - (vf / rho_max) * rho**2
q_prime = rho_prime * vf2 - (vf2 / rho_max2) * rho_prime**2
q3 = rho3 * vf3 - (vf3 / rho_max3) * rho3**2
q4 = rho4 * vf4 - (vf4 / rho_max4) * rho4**2

# 绘制图像
plt.figure(figsize=(10, 9))
plt.plot(rho, q, label="Normal segment flow", color='b')
plt.plot(rho_prime, q_prime, label="Bottleneck segment flow", color='g')
plt.plot(rho3, q3, label="Normal segment flow with platooning", color='b', linestyle='-.')
plt.plot(rho4, q4, label="Bottleneck segment flow with platooning", color='g', linestyle='-.')



# 设置坐标轴范围
plt.xlim(0, max(rho_max, rho_max2, rho_max3, rho_max4) * 1.1)
plt.ylim(0, max(max(q), max(q_prime), max(q3), max(q4)) * 1.3)

# 设置坐标轴标签
plt.xlabel(r'Density $\rho$ $(veh/km)$', fontsize=18, labelpad=15)
plt.ylabel(r'Flow $q$ $(veh/h)$', fontsize=18, labelpad=30)

# 添加图例
plt.legend()

# special point
q_max_1 = vf * rho_max / 4
q_max_2 = vf2 * rho_max2 / 4
q_max_3 = vf3 * rho_max3 / 4
q_max_4 = vf4 * rho_max4 / 4
point1_x = 70
point1_y = point1_x * vf - (vf / rho_max) * point1_x**2

point2_y = (rho_max2 / 2) * vf2 - (vf2 / rho_max2) * (rho_max2 / 2)**2
point2_x = (rho_max / 2) * (1 + math.sqrt(1 - (4 * point2_y) / (rho_max * vf)))

point3_x = 130
point3_y = point3_x * vf - (vf / rho_max) * point3_x**2

point5_x = rho_max4 / 2
point5_y = point5_x * vf4 - (vf4 / rho_max4) * point5_x**2

point4_y = point5_y
point4_x =  (rho_max3 / 2) * (1 + math.sqrt(1 - (4 * point4_y) / (rho_max3 * vf3)))
point4_x = rho_max3/2 - (point4_x - rho_max3/2)


plt.annotate(r'$\rho_{max}$', xy=(rho_max, 0), xytext=(rho_max - 5 , -250), fontsize=12)
plt.annotate(r'$\rho_c$', xy=(rho_max/2, 0), xytext=(rho_max/2-2, -280), fontsize=12)
plt.annotate(r'$q_{max}$', xy=(0, q_max_1), xytext=(-13, q_max_1 - 50), fontsize=12)
plt.annotate(r'$\rho_{max}^{p}$', xy=(rho_max3, 0), xytext=(rho_max3 - 3, -275), fontsize=12)
plt.annotate(r'$\rho_c^p$', xy=(rho_max3/2, 0), xytext=(rho_max3/2-2, -275), fontsize=12)
plt.annotate(r'$q_{max}^p$', xy=(0, q_max_3), xytext=(-12, q_max_3 - 50), fontsize=12)
plt.annotate(r'$q_{initial}$', xy=(0, point1_y), xytext=(-14, point1_y - 50), fontsize=12)
plt.annotate(r'$\frac{2}{3}q_{max}^{p}$', xy=(0, point4_y), xytext=(-15, point4_y - 50), fontsize=12)
plt.annotate(r'$\frac{2}{3}q_{max}$', xy=(0, point2_y), xytext=(-15, point2_y - 50), fontsize=12)

plt.annotate(r'1', xy=(point1_x, point1_y), xytext=(point1_x, point1_y - 300))
plt.annotate(r'2', xy=(point2_x, point2_y), xytext=(point2_x + 2, point2_y ))
plt.annotate(r'3', xy=(point3_x, point3_y), xytext=(point3_x - 5, point3_y - 300))
plt.annotate(r'4', xy=(point4_x, point4_y), xytext=(point4_x - 5, point4_y + 100))
plt.annotate(r'5', xy=(point5_x, point5_y), xytext=(point5_x, point5_y - 300))
plt.annotate(r'6', xy=(point4_x, point4_y), xytext=(point4_x - 8, point4_y - 220))

# plt.scatter(rho_max/2, q_max_1, label='q_max1', color='black', zorder=10)
# plt.scatter(rho_max2/2, q_max_2, label='q_max2',color='black',zorder=10)
# plt.scatter(rho_max3/2, q_max_3, label='q_max3', color='black', zorder=10)
# plt.scatter(rho_max4/2, q_max_4, label='q_max4',color='black',zorder=10)
plt.scatter(point1_x, point1_y, label='point1', color='black', zorder=10)
plt.scatter(point2_x, point2_y, label='point2', color='black', zorder=10)
plt.scatter(point3_x, point3_y, label='point3', color='black', zorder=10)
plt.scatter(point4_x, point4_y, label='point4', color='black', zorder=10)
plt.scatter(point5_x, point5_y, label='point5', color='black', zorder=10)


# special line
plt.plot((0, rho_max/2), (q_max_1,q_max_1), c='grey', linestyle='--', linewidth=1)
plt.plot((rho_max/2, rho_max/2), (0, q_max_1), c='grey', linestyle='--', linewidth=1)
plt.plot((0, rho_max3/2), (q_max_3,q_max_3), c='grey', linestyle='--', linewidth=1)
plt.plot((rho_max3/2, rho_max3/2), (0, q_max_3), c='grey', linestyle='--', linewidth=1)
plt.plot((0, point3_x), (point1_y, point3_y), c='orange', linestyle='--', linewidth=1.2)
plt.plot((0, point5_x), (point4_y, point5_y), c='orange', linestyle='--', linewidth=1.2)
plt.plot((0, point2_x), (point2_y, point2_y), c='orange', linestyle='--', linewidth=1.2)


# plt.gca().axis('off')
plt.gca().xaxis.set_major_formatter(plt.NullFormatter())
plt.gca().yaxis.set_major_formatter(plt.NullFormatter())

# 添加网格线
# plt.grid(True)

# 显示图像
plt.show()
