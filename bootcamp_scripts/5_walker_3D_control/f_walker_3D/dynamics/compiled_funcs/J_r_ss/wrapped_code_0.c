/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                      This file is part of 'autowrap'                       *
 ******************************************************************************/
#include "wrapped_code_0.h"
#include <math.h>

void autofunc(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l1, double l2, double *out_4488021216560093543) {

   out_4488021216560093543[0] = 1;
   out_4488021216560093543[1] = 0;
   out_4488021216560093543[2] = 0;
   out_4488021216560093543[3] = 0;
   out_4488021216560093543[4] = -l1*(1 - cos(pitch_rk))*(((sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw))*sin(roll_rh) + cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-sin(pitch)*sin(yaw)*sin(yaw_rh) - sin(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)) - l1*(((sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw))*sin(roll_rh) + cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-sin(pitch)*sin(yaw)*sin(yaw_rh) - sin(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw)) - w*(1 - cos(yaw_rh))*sin(pitch)*sin(yaw) - w*((sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw))*cos(roll_rh) - sin(roll_rh)*cos(pitch)) - w*sin(pitch)*sin(yaw_rh)*cos(yaw) - w*sin(roll_rh)*cos(pitch) + (-l1 - l2)*(-(((sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw))*sin(roll_rh) + cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-sin(pitch)*sin(yaw)*sin(yaw_rh) - sin(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + (((sin(pitch)*sin(yaw)*cos(yaw_rh) - sin(pitch)*sin(yaw_rh)*cos(yaw))*sin(roll_rh) + cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-sin(pitch)*sin(yaw)*sin(yaw_rh) - sin(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[5] = -l1*(1 - cos(pitch_rk))*((-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(pitch_rh)) - l1*((-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh)) + w*(1 - cos(yaw_rh))*cos(pitch)*cos(yaw) - w*(-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(roll_rh) - w*sin(yaw)*sin(yaw_rh)*cos(pitch) + (-l1 - l2)*(-((-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(pitch_rh))*sin(pitch_rk) + ((-sin(yaw)*sin(yaw_rh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[6] = 0;
   out_4488021216560093543[7] = 0;
   out_4488021216560093543[8] = 0;
   out_4488021216560093543[9] = 0;
   out_4488021216560093543[10] = -l1*(1 - cos(pitch_rk))*((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(roll_rh) - sin(pitch)*sin(roll_rh))*cos(pitch_rh) - l1*((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(roll_rh) - sin(pitch)*sin(roll_rh))*sin(pitch_rh)*sin(pitch_rk) - w*(-(-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) - sin(pitch)*cos(roll_rh)) - w*(-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) - w*sin(pitch)*cos(roll_rh) + (-l1 - l2)*(-((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(roll_rh) - sin(pitch)*sin(roll_rh))*sin(pitch_rh)*sin(pitch_rk) + ((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(roll_rh) - sin(pitch)*sin(roll_rh))*cos(pitch_rh)*cos(pitch_rk));
   out_4488021216560093543[11] = -l1*(1 - cos(pitch_rk))*(-((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*sin(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh)) - l1*(((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*cos(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*((-((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*sin(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - (((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*cos(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));
   out_4488021216560093543[12] = -l1*(1 - cos(pitch_rk))*((sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (sin(yaw)*cos(pitch)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(pitch_rh)) - l1*((sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (sin(yaw)*cos(pitch)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh)) - w*(sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(roll_rh) + w*sin(yaw)*sin(yaw_rh)*cos(pitch) + w*cos(pitch)*cos(yaw)*cos(yaw_rh) + (-l1 - l2)*(-((sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (sin(yaw)*cos(pitch)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(pitch_rh))*sin(pitch_rk) + ((sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (sin(yaw)*cos(pitch)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[13] = -l1*(((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*sin(pitch_rh) + (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - l1*(((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*cos(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*(-(((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*sin(pitch_rh) + (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - (((-sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*cos(roll_rh))*cos(pitch_rh) - (sin(yaw)*sin(yaw_rh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));
   out_4488021216560093543[14] = 0;
   out_4488021216560093543[15] = 1;
   out_4488021216560093543[16] = 0;
   out_4488021216560093543[17] = -l1*(1 - cos(pitch_rk))*((((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh)) - l1*((((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh)) - w*(1 - cos(yaw_rh))*(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) - w*(((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*cos(roll_rh) + sin(roll_rh)*cos(pitch)*cos(roll)) + w*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh) + w*sin(roll_rh)*cos(pitch)*cos(roll) + (-l1 - l2)*(-((((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + ((((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[18] = -l1*(1 - cos(pitch_rk))*(((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*sin(roll)*cos(roll_rh))*cos(pitch_rh) - (sin(roll)*sin(yaw)*sin(yaw_rh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)) - l1*(((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*sin(roll)*cos(roll_rh))*sin(pitch_rh) + (sin(roll)*sin(yaw)*sin(yaw_rh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw)) + w*(1 - cos(yaw_rh))*sin(roll)*sin(yaw)*cos(pitch) - w*((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw))*cos(roll_rh) - sin(pitch)*sin(roll)*sin(roll_rh)) - w*sin(pitch)*sin(roll)*sin(roll_rh) + w*sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw) + (-l1 - l2)*(-(((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*sin(roll)*cos(roll_rh))*sin(pitch_rh) + (sin(roll)*sin(yaw)*sin(yaw_rh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + (((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_rh) + sin(roll)*sin(yaw_rh)*cos(pitch)*cos(yaw))*sin(roll_rh) + sin(pitch)*sin(roll)*cos(roll_rh))*cos(pitch_rh) - (sin(roll)*sin(yaw)*sin(yaw_rh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[19] = -l1*(1 - cos(pitch_rk))*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_rh))*sin(pitch_rh)) - l1*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh)) - w*(1 - cos(yaw_rh))*(-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) - w*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh))*cos(roll_rh) + w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-l1 - l2)*(-(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + (((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[20] = 0;
   out_4488021216560093543[21] = 0;
   out_4488021216560093543[22] = 0;
   out_4488021216560093543[23] = 0;
   out_4488021216560093543[24] = -l1*(1 - cos(pitch_rk))*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(roll_rh) + sin(roll)*sin(roll_rh)*cos(pitch))*cos(pitch_rh) - l1*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(roll_rh) + sin(roll)*sin(roll_rh)*cos(pitch))*sin(pitch_rh)*sin(pitch_rk) - w*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) + sin(roll)*cos(pitch)*cos(roll_rh)) - w*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) + w*sin(roll)*cos(pitch)*cos(roll_rh) + (-l1 - l2)*(-(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(roll_rh) + sin(roll)*sin(roll_rh)*cos(pitch))*sin(pitch_rh)*sin(pitch_rk) + (((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(roll_rh) + sin(roll)*sin(roll_rh)*cos(pitch))*cos(pitch_rh)*cos(pitch_rk));
   out_4488021216560093543[25] = -l1*(1 - cos(pitch_rk))*(-(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh)) - l1*((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*((-(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - ((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));
   out_4488021216560093543[26] = -l1*(1 - cos(pitch_rk))*((-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(pitch_rh)) - l1*((-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh)) - w*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(roll_rh) - w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh) + (-l1 - l2)*(-((-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + ((-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[27] = -l1*((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - l1*((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*(-((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - ((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));
   out_4488021216560093543[28] = 0;
   out_4488021216560093543[29] = 0;
   out_4488021216560093543[30] = 1;
   out_4488021216560093543[31] = -l1*(1 - cos(pitch_rk))*((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh)) - l1*((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh)) - w*(1 - cos(yaw_rh))*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) - w*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*cos(roll_rh) + sin(roll)*sin(roll_rh)*cos(pitch)) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh) + w*sin(roll)*sin(roll_rh)*cos(pitch) + (-l1 - l2)*(-((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*sin(pitch_rh) + (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + ((((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_rh))*sin(roll_rh) - sin(roll)*cos(pitch)*cos(roll_rh))*cos(pitch_rh) - (-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[32] = -l1*(1 - cos(pitch_rk))*(((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_rh) - sin(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-sin(yaw)*sin(yaw_rh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh)) - l1*(((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_rh) - sin(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-sin(yaw)*sin(yaw_rh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw)) - w*(1 - cos(yaw_rh))*sin(yaw)*cos(pitch)*cos(roll) - w*((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw))*cos(roll_rh) + sin(pitch)*sin(roll_rh)*cos(roll)) + w*sin(pitch)*sin(roll_rh)*cos(roll) - w*sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw) + (-l1 - l2)*(-(((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_rh) - sin(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-sin(yaw)*sin(yaw_rh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + (((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_rh) - sin(yaw_rh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_rh) - sin(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-sin(yaw)*sin(yaw_rh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[33] = -l1*(1 - cos(pitch_rk))*(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(pitch_rh)) - l1*(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh)) - w*(1 - cos(yaw_rh))*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw)) - w*((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*cos(roll_rh) + w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-l1 - l2)*(-(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + (((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[34] = 0;
   out_4488021216560093543[35] = 0;
   out_4488021216560093543[36] = 0;
   out_4488021216560093543[37] = 0;
   out_4488021216560093543[38] = -l1*(1 - cos(pitch_rk))*(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(roll_rh) - sin(roll_rh)*cos(pitch)*cos(roll))*cos(pitch_rh) - l1*(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(roll_rh) - sin(roll_rh)*cos(pitch)*cos(roll))*sin(pitch_rh)*sin(pitch_rk) - w*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - cos(pitch)*cos(roll)*cos(roll_rh)) - w*((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) - w*cos(pitch)*cos(roll)*cos(roll_rh) + (-l1 - l2)*(-(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(roll_rh) - sin(roll_rh)*cos(pitch)*cos(roll))*sin(pitch_rh)*sin(pitch_rk) + (((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(roll_rh) - sin(roll_rh)*cos(pitch)*cos(roll))*cos(pitch_rh)*cos(pitch_rk));
   out_4488021216560093543[39] = -l1*(1 - cos(pitch_rk))*(-(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh)) - l1*((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*((-(((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - ((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));
   out_4488021216560093543[40] = -l1*(1 - cos(pitch_rk))*((-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(pitch_rh)) - l1*((-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) - w*(1 - cos(roll_rh))*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh)) - w*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*cos(roll_rh) - w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + w*(-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh) + (-l1 - l2)*(-((-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh)*sin(roll_rh) + (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*cos(pitch_rh))*sin(pitch_rk) + ((-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(roll_rh)*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(pitch_rh))*cos(pitch_rk));
   out_4488021216560093543[41] = -l1*((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - l1*((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk) + (-l1 - l2)*(-((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*sin(pitch_rh) + (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*cos(pitch_rh))*cos(pitch_rk) - ((((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_rh))*sin(roll_rh) + cos(pitch)*cos(roll)*cos(roll_rh))*cos(pitch_rh) - (-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_rh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_rh))*sin(pitch_rh))*sin(pitch_rk));

}