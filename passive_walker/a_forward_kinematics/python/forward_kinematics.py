import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics import Integrator

l1 = 1
l2 = 1
theta1 = 30 / 180 * np.pi
theta2 = 90 / 180 * np.pi


