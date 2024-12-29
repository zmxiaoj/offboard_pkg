import numpy as np
import matplotlib.pyplot as plt

# Parameters
t = np.linspace(0, 4*np.pi, 1000)
width = 3.0
length = 1.0
omega = 1.0

# Position
x = width * np.sin(omega * t)
y = length * np.sin(2 * omega * t)

# Velocity
dx = width * omega * np.cos(omega * t)
dy = 2 * length * omega * np.cos(2 * omega * t)

# Plot trajectory
plt.figure(figsize=(10, 5))
plt.subplot(121)
plt.plot(x, y, 'b-', label='Trajectory')
plt.quiver(x[::50], y[::50], dx[::50], dy[::50], 
          scale=50, color='r', label='Velocity')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('8-Shape Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()