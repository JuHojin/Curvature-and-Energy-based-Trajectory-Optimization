import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

def calculate_curvature(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = np.abs(ddx * dy - dx * ddy) / (dx**2 + dy**2)**1.5
    curvature = np.nan_to_num(curvature)
    return curvature

def get_circumcenter(x1, y1, x2, y2, x3, y3):
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if d == 0:
        return np.inf, np.inf
    else:
        ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / d
        uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / d
        return ux, uy

def calculate_curvature_by_turning_radius(x, y):
    curvatures = []
    for i in range(1, len(x) - 1):
        x1, y1 = x[i - 1], y[i - 1]
        x2, y2 = x[i], y[i]
        x3, y3 = x[i + 1], y[i + 1]

        cx, cy = get_circumcenter(x1, y1, x2, y2, x3, y3)

        if np.isinf(cx) or np.isinf(cy):
            curvature = 0.0
        else:
            R = np.sqrt((x2 - cx)**2 + (y2 - cy)**2)
            if R != 0.0:
                curvature = 1.0 / R
            else:
                curvature = 0.0

        curvatures.append(curvature)

    curvatures = [0] + curvatures + [0]
    return np.array(curvatures)

def apply_bspline(x, y, smooth_factor=0.001):
    tck, u = splprep([x, y], s=smooth_factor)
    u_new = np.linspace(0, 1, len(x) * 1)
    x_smooth, y_smooth = splev(u_new, tck)
    return x_smooth, y_smooth

data_path = os.path.join(os.path.dirname(__file__), '../data/final_plan.csv')
data = pd.read_csv(os.path.abspath(data_path))

if 'x' not in data.columns or 'y' not in data.columns:
    raise ValueError("The CSV file does not contain 'x' or 'y' columns.")

x = data['x'].values
y = data['y'].values

# Gradient method and turning radius method curvature calculations
curvature = calculate_curvature(x, y)
curvature_by_turning_radius = calculate_curvature_by_turning_radius(x, y)

x_smooth, y_smooth = apply_bspline(x, y)

# Gradient method and turning radius method curvature calculations for the B-spline path
curvature_smooth = calculate_curvature(x_smooth, y_smooth)
curvature_by_turning_radius_smooth = calculate_curvature_by_turning_radius(x_smooth, y_smooth)

plt.figure(figsize=(12, 12))
# Gradient method curvature plots
plt.subplot(2, 1, 1)
plt.plot(curvature, label='Gradient Method (Original Path)', color='blue')
plt.plot(np.linspace(0, len(curvature) - 1, len(curvature_smooth)), curvature_smooth, label='Gradient Method (B-spline Path)', color='red', linestyle='--')
plt.title('Curvature Plot - Gradient Method')
plt.xlabel('Index')
plt.ylabel('Curvature')
plt.legend()

# Turning radius method curvature plots
plt.subplot(2, 1, 2)
plt.plot(curvature_by_turning_radius, label='Turning Radius Method (Original Path)', color='orange')
plt.plot(np.linspace(0, len(curvature_by_turning_radius) - 1, len(curvature_by_turning_radius_smooth)), curvature_by_turning_radius_smooth, label='Turning Radius Method (B-spline Path)', color='green', linestyle='--')
plt.title('Curvature Plot - Turning Radius Method')
plt.xlabel('Index')
plt.ylabel('Curvature')
plt.legend()

plt.tight_layout()
plt.show()
