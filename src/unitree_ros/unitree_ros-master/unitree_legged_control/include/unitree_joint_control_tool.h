/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
// #define _UNITREE_JOINT_CONTROL_TOOL_H_
#ifndef _LAIKAGO_CONTROL_TOOL_H_
#define _LAIKAGO_CONTROL_TOOL_H_

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <math.h>

#define posStopF (2.146E+9f)  // stop position control mode
#define velStopF (16000.0f)   // stop velocity control mode

typedef struct 
{
    uint8_t mode;
    double pos;
    double posStiffness;
    double vel;
    double velStiffness;
    double torque;
} ServoCmd;

// double clamp(double&, double, double);  // eg. clamp(1.5, -1, 1) = 1
// double computeVel(double current_position, double last_position, double last_velocity, double duration);  // get current velocity
// double computeTorque(double current_position, double current_velocity, ServoCmd&);  // get torque

float clamp(float &val, float min_val, float max_val)
{
    val = std::min(std::max(val, min_val), max_val);
}

double clamp(double &val, double min_val, double max_val)
{
    val = std::min(std::max(val, min_val), max_val);
}

double computeVel(double currentPos, double lastPos, double lastVel, double period)
{
    //return lastVel*0.0f + 1.0f*(currentPos-lastPos)/period;
    return lastVel*0.35f + 0.65f*(currentPos-lastPos)/period;
}

double computeTorque(double currentPos, double currentVel, ServoCmd &cmd)
{
    double targetPos, targetVel, targetTorque, posStiffness, velStiffness, calcTorque;
    targetPos = cmd.pos;
    targetVel = cmd.vel;
    targetTorque = cmd.torque;
    posStiffness = cmd.posStiffness;
    velStiffness = cmd.velStiffness;
    if(fabs(targetPos-posStopF) < 1e-6) posStiffness = 0;
    if(fabs(targetVel-velStopF) < 1e-6) velStiffness = 0;
    // clamp(targetVel, -53.0f, 53.0f);
    calcTorque = posStiffness*(targetPos-currentPos) + velStiffness*(targetVel-currentVel) + targetTorque;
    // clamp(calcTorque, -55.0f, 55.0f);
    return calcTorque;
}

#endif
