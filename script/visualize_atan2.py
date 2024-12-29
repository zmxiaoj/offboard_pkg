import numpy as np
import matplotlib.pyplot as plt

# Generate points
x = np.linspace(-5, 5, 1000)
y = np.array([1.0])  # 固定y值
angles = []

# 计算不同x值对应的atan2值
for x_val in x:
    angle = np.arctan2(y, x_val)
    angles.append(angle[0])

# 创建图形
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

# Plot 1: atan2曲线
ax1.plot(x, angles, 'b-', label='atan2(1,x)')
ax1.axhline(y=0, color='k', linestyle='-', alpha=0.3)
ax1.axvline(x=0, color='k', linestyle='-', alpha=0.3)
ax1.grid(True)
ax1.set_title('atan2(1,x) Function')
ax1.set_xlabel('x')
ax1.set_ylabel('angle (radians)')
ax1.legend()

# Plot 2: 极坐标表示
theta = np.linspace(0, 2*np.pi, 100)
r = np.ones_like(theta)
ax2.plot(r*np.cos(theta), r*np.sin(theta), 'r--')
ax2.quiver(0, 0, 1, 0, angles='xy', scale_units='xy', scale=1)
ax2.set_aspect('equal')
ax2.grid(True)
ax2.set_title('Unit Circle')

plt.show()