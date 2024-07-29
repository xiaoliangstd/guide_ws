/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// reference: https://github.com/Teddy-Liao/walk-these-ways-go2

#include <iostream>
#include "FSM/State_RlWtw.h"

State_RlWtw::State_RlWtw(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::RLWTW, "wtw RL deploy")
{
    int ret1 = pthread_mutex_init(&write_cmd_mutex,NULL);        
    loadPolicy();
    lastvelCmd.setZero();
}

void State_RlWtw::loadPolicy()
{
   std::string policy_file_path;
#ifdef ROBOT_TYPE_Go2
		policy_file_path = getWorkingDir() + "/src/unitree_guide/unitree_guide/policy/liang_wtw_go2_test2.mnn";
#endif
    std::cout<<"[State_RlWtw] policy_file_path: "<<policy_file_path<<std::endl;

    net = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(policy_file_path.c_str()));
    MNN::ScheduleConfig config;
    config.numThread = 2;
    session = net->createSession(config);
    obs_tensor = net->getSessionInput(session, NULL);
    act_tensor = net->getSessionOutput(session, NULL);
    obs_temp_mnn = MNN::Tensor::create<float>(obs_tensor->shape(), NULL, MNN::Tensor::CAFFE);

    decimation = 1;
    clip_actions = 10.0;
    clip_obs = 10.0; 
    this->dt = 0.02; 
    this->kp = 22; 
    this->kd = 0.5; 
    
    obs_history_steps = 30;
    obs_dim = obs_tensor->shape().back() / obs_history_steps;
    act_dim = act_tensor->shape().back();
    obs_buf = obsHistory(obs_history_steps,obs_dim);
    obs_mean.setZero(obs_dim);
    obs_scales.setZero(obs_dim);
    obs_scaled.setZero(obs_dim);
    obs_current.setZero(obs_dim);
    act_mean.setZero(act_dim);
    act_std.setZero(act_dim);
    act_prev.setZero(act_dim);
    act_prev_prev.setZero(act_dim);
    act_scaled.setZero(act_dim);
    act_temp.setZero(act_dim);
    dof_vel_limits.setZero(act_dim);
    torque_limits.setZero(act_dim);


    for(int i=0;i<obs_dim;i++)
    {
        obs_mean(i) = 0;
    }
    // action rate
    obs_mean(18) = 0.1;  // FL_hip_joint
    obs_mean(19) = 0.8; // FL_thigh_joint
    obs_mean(20) = -1.5; // FL_calf_joint
    obs_mean(21) = -0.1; // FR_hip_joint
    obs_mean(22) = 0.8; // FR_thigh_joint
    obs_mean(23) = -1.5; // FR_calf_joint
    obs_mean(24) = 0.1; // RL_hip_joint
    obs_mean(25) = 1.0; // RL_thigh_joint
    obs_mean(26) = -1.5; // RL_calf_joint
    obs_mean(27) = -0.1; // RR_hip_joint
    obs_mean(28) = 1.0; // RR_thigh_joint
    obs_mean(29) = -1.5; // RR_calf_joint


    for(int i=0;i<obs_dim;i++)
    {
        obs_scales(i) = 1.0;
    }
    obs_scales(0) = 1.0;  // obs_scales - projected_gravity
    obs_scales(1) = 1.0;
    obs_scales(2) = 1.0;

    // commands_scale
    obs_scales(3) = 2.0; // obs_scales.lin_vel
    obs_scales(4) = 2.0; // obs_scales.lin_vel
    obs_scales(5) = 0.25; // obs_scales.ang_vel
    obs_scales(6) = 2.0; // obs_scales.body_height_cmd
    obs_scales(7) = 1.0; // obs_scales.gait_freq_cmd
    obs_scales(8) = 1.0; // obs_scales.gait_phase_cmd
    obs_scales(9) = 1.0; // obs_scales.gait_phase_cmd
    obs_scales(10) = 1.0; // obs_scales.gait_phase_cmd
    obs_scales(11) = 1.0; // obs_scales.gait_phase_cmd
    obs_scales(12) = 0.15; // obs_scales.footswing_height_cmd
    obs_scales(13) = 0.3; // obs_scales.body_pitch_cmd
    obs_scales(14) = 0.3; // obs_scales.body_roll_cmd
    obs_scales(15) = 1.0; // obs_scales.stance_width_cmd
    obs_scales(16) = 1.0; // obs_scales.stance_length_cmd
    obs_scales(17) = 1.0; // obs_scales.aux_reward_cmd

    // obs_scales.dof_pos
    obs_scales.segment<12>(18) << act_prev; // last_action
    for(int i=18;i<30;i++)
    {
        obs_scales(i) = 1.0;
    }
    // dof_vel
    for(int i=30;i<42;i++)
    {
        obs_scales(i) = 0.05;
    }
    // actions
    for(int i=42;i<54;i++)
    {
        obs_scales(i) = 1.0;
    }
    // last_actions
    for(int i=54;i<66;i++)
    {
        obs_scales(i) = 1.0;
    }
    // clock_inputs
    for(int i=66;i<70;i++)
    {
        obs_scales(i) = 1.0;
    }

    // action rate
    act_mean(0) = 0.1;  // FL_hip_joint
    act_mean(1) = 0.8; // FL_thigh_joint
    act_mean(2) = -1.5; // FL_calf_joint

    act_mean(3) = -0.1; // FR_hip_joint
    act_mean(4) = 0.8; // FR_thigh_joint
    act_mean(5) = -1.5; // FR_calf_joint

    act_mean(6) = 0.1; // RL_hip_joint
    act_mean(7) = 1.0; // RL_thigh_joint
    act_mean(8) = -1.5; // RL_calf_joint

    act_mean(9) = -0.1; // RR_hip_joint
    act_mean(10) = 1.0; // RR_thigh_joint
    act_mean(11) = -1.5; // RR_calf_joint

    for(int i=0;i<12;i++)
        act_std(i) = 0.25; // action scale

    act_std(0) *=0.5; // hip_scale_reduction
    act_std(3) *=0.5; 
    act_std(6) *=0.5; 
    act_std(9) *=0.5; 

    act_prev << act_mean;
    obs_current << obs_mean;
   
    gait<<0.5,0,0; // trot gait
    gait_duration = 0.5; // s
    gait_frequency = 3; // hz
    footswing_height = 0.05; // m
}

void State_RlWtw::enter()
{
    _ctrlComp->setAllStance();
    for (int i = 0; i < obs_history_steps * decimation; ++i)
    { 
        obs_current.setZero(obs_dim);
        _B2G_RotMat = _lowState->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity_body;
        Vec3 projected_gravity_world;
        projected_gravity_world<< 0,0,-1;
        projected_gravity_body = _G2B_RotMat * projected_gravity_world;
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i) = projected_gravity_body(i);
        }
        obs_current(3) = 0.0; // commands.x_vel_cmd 
        obs_current(4) = 0.0; // commands.y_vel_cmd  
        obs_current(5) = 0.0; // commands.yaw_vel_cmd 
        obs_current(6) = 0.0; // commands.body_height_cmd 
        obs_current(7) = 0.0; // commands.step_frequency_cmd 
        obs_current(8) = gait(0); // commands.gait offset 
        obs_current(9) = gait(1); // commands.gait offset 
        obs_current(10) = gait(2); // commands.gait offset 
        obs_current(11) = gait_duration; // commands.gait_duration 
        obs_current(12) = footswing_height; // commands.footswing_height_cmd
        obs_current(13) = 0.0; // commands.pitch_cmd
        obs_current(14) = 0.0; // commands.roll_cmd
        obs_current(15) = 0.25; // commands.stance_width_cmd
        obs_current(16) = 0.25; 
        obs_current(17) = 0.00427; 

        _q = vec34ToVec12(_lowState->getQ());
        _qd = vec34ToVec12(_lowState->getQd());

        for (int i = 0; i < 12; ++i)
        {
            obs_current(18 + reorder_map.at(i)) = _q(i);
            obs_current(30 + reorder_map.at(i)) = _qd(i);
        }

        act_prev.setZero(); 
        obs_current.segment<12>(42) << act_prev; // last_action

        act_prev.setZero(); 
        obs_current.segment<12>(54) << act_prev; // last_last_action

        // clock
        obs_current(66) = 0; 
        obs_current(67) = 0; 
        obs_current(68) = 0; 
        obs_current(69) = 0; 

        obs_scaled = (obs_current - obs_mean).cwiseProduct(obs_scales).cwiseMax(-clip_obs).cwiseMin(clip_obs);
        obs_buf.insert(obs_scaled);
    }
    threadRunning = true;
    _thread = std::thread(&State_RlWtw::inferenceThreadFunction, this);
}

void State_RlWtw::inferenceThreadFunction()
{
    while(threadRunning)
    {
        _startTime = getSystemTime();
    
        gait_indices(0) += gait_frequency * this->dt; 
        gait_indices(1) += gait_frequency * this->dt; 
        gait_indices(2) += gait_frequency * this->dt; 
        gait_indices(3) += gait_frequency * this->dt;

        gait_indices(0) = fmod(gait_indices(0),1);
        gait_indices(1) = fmod(gait_indices(1),1);
        gait_indices(2) = fmod(gait_indices(2),1);
        gait_indices(3) = fmod(gait_indices(3),1);

        foot_indices(0) = gait_indices(0) + gait(0) + gait(1) + gait(2);
        foot_indices(1) = gait_indices(1) + gait(1);
        foot_indices(2) = gait_indices(2) + gait(2);
        foot_indices(3) = gait_indices(3) + gait(0);

        foot_indices(0) = fmod(foot_indices(0),1);
        foot_indices(1) = fmod(foot_indices(1),1);
        foot_indices(2) = fmod(foot_indices(2),1);
        foot_indices(3) = fmod(foot_indices(3),1);

        clock_inputs(0) = sin(2*M_PI*foot_indices(0));
        clock_inputs(1) = sin(2*M_PI*foot_indices(1));
        clock_inputs(2) = sin(2*M_PI*foot_indices(2));
        clock_inputs(3) = sin(2*M_PI*foot_indices(3));

        
        obs_current.setZero(obs_dim);
        _B2G_RotMat = _lowState->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity_body;
        Vec3 projected_gravity_world;
        projected_gravity_world<< 0,0,-1;
        projected_gravity_body = _G2B_RotMat * projected_gravity_world;
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i) = projected_gravity_body(i);
        }
        _userValue = _lowState->userValue;
        obs_current(3) = _userValue.ly;
        obs_current(4) = -_userValue.lx;
        obs_current(5) = -_userValue.rx;
        obs_current(6) = 0.0; // body_height_cmd
        obs_current(7) = gait_frequency; // gait_frequency
        obs_current(8) = gait(0); // gait offset 
        obs_current(9) = gait(0); // gait offset 
        obs_current(10) = gait(2); // gait offset 
        obs_current(11) = gait_duration; // duration
        obs_current(12) = footswing_height; // footswing_height_cmd
        obs_current(13) = 0.0; // pitch_cmd
        obs_current(14) = 0.0; // roll_cmd
        obs_current(15) = 0.25; // stance_width_cmd
        obs_current(16) = 0.428; // stance_length_cmd
        obs_current(17) = 0.0002137; // aux_reward_cmd

        _q = vec34ToVec12(_lowState->getQ());
        _qd = vec34ToVec12(_lowState->getQd());

        for (int i = 0; i < 12; ++i)
        {
            obs_current(18 + reorder_map.at(i)) = _q(i);
            obs_current(30 + reorder_map.at(i)) = _qd(i);
        }

        obs_current.segment<12>(42) << act_prev;  // last_action
        obs_current.segment<12>(54) << act_prev;  // last_last_action todo

        // clock
        obs_current(66) = clock_inputs(0); 
        obs_current(67) = clock_inputs(1); 
        obs_current(68) = clock_inputs(2); 
        obs_current(69) = clock_inputs(3); 

        obs_scaled = (obs_current - obs_mean).cwiseProduct(obs_scales).cwiseMax(-clip_obs).cwiseMin(clip_obs);

        obs_buf.insert(obs_scaled);
        
        Eigen::VectorXf obs = obs_buf.get_obs_vec();  // temporal observation
        for (int i = 0; i < obs_dim * obs_history_steps; ++i)
        {
            obs_temp_mnn->host<float>()[i] = obs(i);
        }

        obs_tensor->copyFromHostTensor(obs_temp_mnn);
        net->runSession(session);

        for (int i = 0; i < 12; ++i)
        {
            act_prev(i) = act_tensor->host<float>()[i];
        }

        act_prev = act_prev.cwiseMax(-clip_actions).cwiseMin(clip_actions);
        pthread_mutex_lock(&write_cmd_mutex);
        act_scaled = act_prev.cwiseProduct(act_std) + act_mean;
        pthread_mutex_unlock(&write_cmd_mutex);
        absoluteWait(_startTime, (long long)(this->dt * 1000000));
    }
    threadRunning = false;
    std::cout<<"done!"<<std::endl;
}


void State_RlWtw::run()
{
    pthread_mutex_lock(&write_cmd_mutex);
    memcpy(act_temp.data(),act_scaled.data(),act_scaled.size()* sizeof(float));
    pthread_mutex_unlock(&write_cmd_mutex);

    for (int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[reorder_map.at(i)].q = act_temp[i];
        _lowCmd->motorCmd[reorder_map.at(i)].Kp = this->kp;
        _lowCmd->motorCmd[reorder_map.at(i)].Kd = this->kd;
        _lowCmd->motorCmd[reorder_map.at(i)].tau = 0;
    }
}

Eigen::VectorXf State_RlWtw::action_limit(Eigen::Ref<Eigen::VectorXf> action)
{
    Eigen::VectorXf dof_vel_ratio = obs_current.segment<12>(21).cwiseQuotient(dof_vel_limits).cwiseMax(-1.).cwiseMin(1.);
    Eigen::VectorXf torque_limits_hi = torque_limits.cwiseProduct(((Eigen::VectorXf::Ones(12) - dof_vel_ratio) / (1.0 - max_torque_vel_ratio)).cwiseMin(1.).cwiseMax(0.));
    Eigen::VectorXf torque_limits_lo = -torque_limits.cwiseProduct(((dof_vel_ratio + Eigen::VectorXf::Ones(12)) / (1.0 - max_torque_vel_ratio)).cwiseMin(1.).cwiseMax(0.));
    return action.cwiseMin((torque_limits_hi + this->kd * obs_current.segment<12>(21)) / this->kp + obs_current.segment<12>(9)).cwiseMax((torque_limits_lo + this->kd * obs_current.segment<12>(21)) / this->kp + obs_current.segment<12>(9));
}

void State_RlWtw::exit()
{
    act_prev.setZero();
    threadRunning = false;
    _thread.join();
}

FSMStateName State_RlWtw::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::RLWTW;
    }
}