import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-21 13:24:17

def get_J_l_ss(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2):

    J_l_ss = np.array([[1, 0, 0, 0, -l1*(1 - cos(pitch_lk))*((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*cos(pitch_lh) - (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*sin(pitch_lh) + (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw)) + w*(1 - cos(yaw_lh))*sin(pitch)*sin(yaw) + w*((sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*cos(roll_lh) + sin(roll_lh)*cos(pitch)) - w*sin(pitch)*sin(yaw_lh)*cos(yaw) - w*sin(roll_lh)*cos(pitch) + (-l1 - l2)*(-((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*sin(pitch_lh) + (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*cos(pitch_lh) - (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh)) - l1*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh)) - w*(1 - cos(yaw_lh))*cos(pitch)*cos(yaw) + w*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(roll_lh) - w*sin(yaw)*sin(yaw_lh)*cos(pitch) + (-l1 - l2)*(-(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + (-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*cos(pitch_lh) - l1*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*sin(pitch_lh)*sin(pitch_lk) + w*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh)) + w*(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) - w*sin(pitch)*cos(roll_lh) + (-l1 - l2)*(-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*sin(pitch_lh)*sin(pitch_lk) + (-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*cos(pitch_lh)*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh)) - l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh)) - l1*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh)) + w*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(roll_lh) - w*sin(yaw)*sin(yaw_lh)*cos(pitch) + w*cos(pitch)*cos(yaw)*cos(yaw_lh) + (-l1 - l2)*(-(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + (-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh))*cos(pitch_lk)), -l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) + (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) + (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), 0, 0, 0, 0, ],[0, 1, 0, -l1*(1 - cos(pitch_lk))*((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)) - l1*((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + w*(((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll)) + w*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh) + w*sin(roll_lh)*cos(pitch)*cos(roll) + (-l1 - l2)*(-((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*cos(pitch_lh) - (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*sin(pitch_lh) + (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw)) - w*(1 - cos(yaw_lh))*sin(roll)*sin(yaw)*cos(pitch) + w*((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) + sin(pitch)*sin(roll)*sin(roll_lh)) - w*sin(pitch)*sin(roll)*sin(roll_lh) + w*sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw) + (-l1 - l2)*(-((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*sin(pitch_lh) + (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*cos(pitch_lh) - (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + w*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*cos(roll_lh) + w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-l1 - l2)*(-(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*cos(pitch_lh) - l1*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*sin(pitch_lh)*sin(pitch_lk) + w*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh)) + w*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) + w*sin(roll)*cos(pitch)*cos(roll_lh) + (-l1 - l2)*(-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*sin(pitch_lh)*sin(pitch_lk) + (-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*cos(pitch_lh)*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh)) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh)) + w*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(roll_lh) + w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh) + (-l1 - l2)*(-(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), 0, 0, 0, 0, ],[0, 0, 1, -l1*(1 - cos(pitch_lk))*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + w*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) - sin(roll)*sin(roll_lh)*cos(pitch)) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh) + w*sin(roll)*sin(roll_lh)*cos(pitch) + (-l1 - l2)*(-((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw)) + w*(1 - cos(yaw_lh))*sin(yaw)*cos(pitch)*cos(roll) + w*((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh)*cos(roll)) + w*sin(pitch)*sin(roll_lh)*cos(roll) - w*sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw) + (-l1 - l2)*(-((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh)) + w*(1 - cos(yaw_lh))*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw)) + w*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(roll_lh) + w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-l1 - l2)*(-(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*cos(pitch_lh) - l1*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*sin(pitch_lh)*sin(pitch_lk) + w*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh)) + w*((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - w*cos(pitch)*cos(roll)*cos(roll_lh) + (-l1 - l2)*(-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*sin(pitch_lh)*sin(pitch_lk) + (-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*cos(pitch_lh)*cos(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh)) - l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), -l1*(1 - cos(pitch_lk))*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh)) + w*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(roll_lh) + w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + w*(-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh) + (-l1 - l2)*(-(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk)), -l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk)), 0, 0, 0, 0, ],])  

    return J_l_ss