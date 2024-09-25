import numpy as np
from scipy.interpolate import RegularGridInterpolator
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics import Integrator as inte, RobotUtils as util

# basic parameters for a walker
hip_m = 1.0 # kg, mass of hip
leg_m = 0.5 # kg, mass of leg
leg_I = 0.02 # kg x m^2, moment of inertia of leg
leg_l = 1.0 # kg x m^2, length of 
leg_c = 0.5 # m, CoM of the leg
g = 10.0 # gravity
slope_angle = 0.01

A_vals = np.linspace(0, 1, 11)  # e.g., 0.0, 0.1, 0.2, ..., 1.0
B_vals = np.linspace(0, 1, 11)
C_vals = np.linspace(0, 1, 11)
D_vals = np.linspace(0, 1, 11)

# Example: Precompute output values for each combination of A, B, C, D
# Here we are assuming some function f(A, B, C, D) that gives the output
def f(A, B, C, D):
    # Example function for output, replace with your real calculation
    return A * B + C * D

# Generate the output values for all combinations
output_values = np.zeros((len(A_vals), len(B_vals), len(C_vals), len(D_vals)))
for i, A in enumerate(A_vals):
    for j, B in enumerate(B_vals):
        for k, C in enumerate(C_vals):
            for l, D in enumerate(D_vals):
                output_values[i, j, k, l] = f(A, B, C, D)

# Create an interpolator object
lut_interpolator = RegularGridInterpolator((A_vals, B_vals, C_vals, D_vals), output_values)

# Define a function to get interpolated value for continuous inputs
def get_continuous_value(A, B, C, D):
    input_point = np.array([A, B, C, D])
    return lut_interpolator(input_point)

# Example usage:
A_input = 0.45
B_input = 0.65
C_input = 0.25
D_input = 0.75
output = get_continuous_value(A_input, B_input, C_input, D_input)
print(output)
