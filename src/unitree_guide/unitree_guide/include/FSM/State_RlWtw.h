/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef RLWTW_H
#define RLWTW_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
// #include <filesystem>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>
#include <thread>
#include "common/getWorkingDir.h"
#include "FSM/FSMState.h"
#include <MNN/Interpreter.hpp>
#include "common/obsHistory.h"

class State_RlWtw : public FSMState
{
public:
    State_RlWtw(CtrlComponents *ctrlComp);
    ~State_RlWtw() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    long long _startT; // unit: us
    double _passT;

    Vec12 _q;
    Vec12 _qd;
    RotMat _B2G_RotMat, _G2B_RotMat;

    void loadPolicy();

    std::shared_ptr<MNN::Interpreter> net = nullptr;
    MNN::Session* session = nullptr;
    MNN::Tensor *obs_tensor = nullptr;
    MNN::Tensor *act_tensor = nullptr;
    MNN::Tensor *obs_temp_mnn = nullptr;

    int obs_dim,act_dim, obs_history_steps, decimation;
    float clip_actions, clip_obs;
    float max_torque_vel_ratio;
    float dt, kp, kd;
    Eigen::VectorXf dof_vel_limits;
    Eigen::VectorXf torque_limits;
    Eigen::VectorXf obs_mean;
    Eigen::VectorXf obs_scales;
    Eigen::VectorXf obs_scaled;
    Eigen::VectorXf act_mean;
    Eigen::VectorXf act_std;
    Eigen::VectorXf act_scaled;
    Eigen::VectorXf act_temp;
    Eigen::VectorXf obs_current;
    Eigen::VectorXf act_prev;
    Eigen::VectorXf act_prev_prev;

    std::array<size_t, 12> reorder_map = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    obsHistory obs_buf;
    float vx_scale, vy_scale, omega_scale;
    
    std::array<float, 3> rpy, gyro, projected_gravity;
    std::array<float, 4> quat;

    Eigen::VectorXf action_limit(Eigen::Ref<Eigen::VectorXf> action);

    std::thread _thread;
    void inferenceThreadFunction();
    bool threadRunning = true;
    long long _startTime;
    pthread_mutex_t write_cmd_mutex;
    bool firstRun = true;

    Vec3 velCmd;
    Vec3 lastvelCmd;

    Vec4 clock_inputs;
    Vec4 foot_indices;
    Vec4 gait_indices;
    Vec3 gait;
    float gait_duration;
    float gait_frequency;
    float footswing_height;
};
#endif // RLWTW_H