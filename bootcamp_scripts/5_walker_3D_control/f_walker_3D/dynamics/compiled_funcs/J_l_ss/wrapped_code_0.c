/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                      This file is part of 'autowrap'                       *
 ******************************************************************************/
#include "wrapped_code_0.h"
#include <math.h>

void autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double w, double l1, double l2, double *out_5241214194046891435) {

   out_5241214194046891435[0] = 1;
   out_5241214194046891435[1] = 0;
   out_5241214194046891435[2] = 0;
   out_5241214194046891435[3] = 0;
   out_5241214194046891435[4] = -l1*(1 - cos(pitch_lk))*((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*cos(pitch_lh) - (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*sin(pitch_lh) + (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw)) + w*(1 - cos(yaw_lh))*sin(pitch)*sin(yaw) + w*((sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*cos(roll_lh) + sin(roll_lh)*cos(pitch)) - w*sin(pitch)*sin(yaw_lh)*cos(yaw) - w*sin(roll_lh)*cos(pitch) + (-l1 - l2)*(-((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*sin(pitch_lh) + (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(sin(pitch)*sin(yaw)*cos(yaw_lh) + sin(pitch)*sin(yaw_lh)*cos(yaw))*sin(roll_lh) + cos(pitch)*cos(roll_lh))*cos(pitch_lh) - (sin(pitch)*sin(yaw)*sin(yaw_lh) - sin(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[5] = -l1*(1 - cos(pitch_lk))*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh)) - l1*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh)) - w*(1 - cos(yaw_lh))*cos(pitch)*cos(yaw) + w*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(roll_lh) - w*sin(yaw)*sin(yaw_lh)*cos(pitch) + (-l1 - l2)*(-(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + (-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[6] = -l1*(1 - cos(pitch_lk))*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*cos(pitch_lh) - l1*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*sin(pitch_lh)*sin(pitch_lk) + w*(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh)) + w*(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) - w*sin(pitch)*cos(roll_lh) + (-l1 - l2)*(-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*sin(pitch_lh)*sin(pitch_lk) + (-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh))*cos(pitch_lh)*cos(pitch_lk));
   out_5241214194046891435[7] = -l1*(1 - cos(pitch_lk))*(-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh)) - l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[8] = -l1*(1 - cos(pitch_lk))*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh)) - l1*(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh)) + w*(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(roll_lh) - w*sin(yaw)*sin(yaw_lh)*cos(pitch) + w*cos(pitch)*cos(yaw)*cos(yaw_lh) + (-l1 - l2)*(-(-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(pitch_lh))*sin(pitch_lk) + (-(sin(yaw)*sin(yaw_lh)*cos(pitch) - cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - (-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[9] = -l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) + (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*sin(pitch_lh) + (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-(-sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*cos(roll_lh))*cos(pitch_lh) - (-sin(yaw)*sin(yaw_lh)*cos(pitch) + cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[10] = 0;
   out_5241214194046891435[11] = 0;
   out_5241214194046891435[12] = 0;
   out_5241214194046891435[13] = 0;
   out_5241214194046891435[14] = 0;
   out_5241214194046891435[15] = 1;
   out_5241214194046891435[16] = 0;
   out_5241214194046891435[17] = -l1*(1 - cos(pitch_lk))*((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)) - l1*((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + w*(((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll)) + w*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh) + w*sin(roll_lh)*cos(pitch)*cos(roll) + (-l1 - l2)*(-((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[18] = -l1*(1 - cos(pitch_lk))*((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*cos(pitch_lh) - (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*sin(pitch_lh) + (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw)) - w*(1 - cos(yaw_lh))*sin(roll)*sin(yaw)*cos(pitch) + w*((-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*cos(roll_lh) + sin(pitch)*sin(roll)*sin(roll_lh)) - w*sin(pitch)*sin(roll)*sin(roll_lh) + w*sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw) + (-l1 - l2)*(-((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*sin(pitch_lh) + (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(-sin(roll)*sin(yaw)*cos(pitch)*cos(yaw_lh) - sin(roll)*sin(yaw_lh)*cos(pitch)*cos(yaw))*sin(roll_lh) + sin(pitch)*sin(roll)*cos(roll_lh))*cos(pitch_lh) - (-sin(roll)*sin(yaw)*sin(yaw_lh)*cos(pitch) + sin(roll)*cos(pitch)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[19] = -l1*(1 - cos(pitch_lk))*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + w*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*cos(roll_lh) + w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-l1 - l2)*(-(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) + (-sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[20] = -l1*(1 - cos(pitch_lk))*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*cos(pitch_lh) - l1*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*sin(pitch_lh)*sin(pitch_lk) + w*(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh)) + w*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) + w*sin(roll)*cos(pitch)*cos(roll_lh) + (-l1 - l2)*(-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*sin(pitch_lh)*sin(pitch_lk) + (-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) + sin(roll)*sin(roll_lh)*cos(pitch))*cos(pitch_lh)*cos(pitch_lk));
   out_5241214194046891435[21] = -l1*(1 - cos(pitch_lk))*(-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh)) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[22] = -l1*(1 - cos(pitch_lk))*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh)) + w*(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(roll_lh) + w*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh) + (-l1 - l2)*(-(-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[23] = -l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[24] = 0;
   out_5241214194046891435[25] = 0;
   out_5241214194046891435[26] = 0;
   out_5241214194046891435[27] = 0;
   out_5241214194046891435[28] = 0;
   out_5241214194046891435[29] = 0;
   out_5241214194046891435[30] = 1;
   out_5241214194046891435[31] = -l1*(1 - cos(pitch_lk))*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh)) - l1*((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh)) + w*(1 - cos(yaw_lh))*(-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + w*(((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*cos(roll_lh) - sin(roll)*sin(roll_lh)*cos(pitch)) + w*(sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh) + w*sin(roll)*sin(roll_lh)*cos(pitch) + (-l1 - l2)*(-((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*sin(pitch_lh) + ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(yaw_lh) - (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(yaw_lh))*sin(roll_lh) - sin(roll)*cos(pitch)*cos(roll_lh))*cos(pitch_lh) - ((-sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[32] = -l1*(1 - cos(pitch_lk))*((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh)) - l1*((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw)) + w*(1 - cos(yaw_lh))*sin(yaw)*cos(pitch)*cos(roll) + w*((sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*cos(roll_lh) - sin(pitch)*sin(roll_lh)*cos(roll)) + w*sin(pitch)*sin(roll_lh)*cos(roll) - w*sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw) + (-l1 - l2)*(-((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + ((-(sin(yaw)*cos(pitch)*cos(roll)*cos(yaw_lh) + sin(yaw_lh)*cos(pitch)*cos(roll)*cos(yaw))*sin(roll_lh) - sin(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - (sin(yaw)*sin(yaw_lh)*cos(pitch)*cos(roll) - cos(pitch)*cos(roll)*cos(yaw)*cos(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[33] = -l1*(1 - cos(pitch_lk))*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh)) + w*(1 - cos(yaw_lh))*(sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw)) + w*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*cos(roll_lh) + w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-l1 - l2)*(-(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) + (sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[34] = -l1*(1 - cos(pitch_lk))*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*cos(pitch_lh) - l1*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*sin(pitch_lh)*sin(pitch_lk) + w*(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh)) + w*((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) - w*cos(pitch)*cos(roll)*cos(roll_lh) + (-l1 - l2)*(-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*sin(pitch_lh)*sin(pitch_lk) + (-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(roll_lh) - sin(roll_lh)*cos(pitch)*cos(roll))*cos(pitch_lh)*cos(pitch_lk));
   out_5241214194046891435[35] = -l1*(1 - cos(pitch_lk))*(-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh)) - l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*((-(-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[36] = -l1*(1 - cos(pitch_lk))*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh)) - l1*(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + w*(1 - cos(roll_lh))*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh)) + w*(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(roll_lh) + w*(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + w*(-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh) + (-l1 - l2)*(-(-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh)*sin(roll_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*cos(pitch_lh))*sin(pitch_lk) + (-(-(sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(roll_lh)*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(pitch_lh))*cos(pitch_lk));
   out_5241214194046891435[37] = -l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - l1*((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk) + (-l1 - l2)*(-((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*sin(pitch_lh) + ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*cos(pitch_lh))*cos(pitch_lk) - ((-((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(yaw_lh) - (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(yaw_lh))*sin(roll_lh) + cos(pitch)*cos(roll)*cos(roll_lh))*cos(pitch_lh) - ((sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*sin(yaw_lh) + (-sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(yaw_lh))*sin(pitch_lh))*sin(pitch_lk));
   out_5241214194046891435[38] = 0;
   out_5241214194046891435[39] = 0;
   out_5241214194046891435[40] = 0;
   out_5241214194046891435[41] = 0;

}