/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_ThreefootStand.h"

State_ThreefootStand::State_ThreefootStand(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::THREEFOOTSTAND, "three foot stand"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(150, 150, 150).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200;
    _Kdw = Vec3(30, 30, 30).asDiagonal();

}

void State_ThreefootStand::enter(){
    _pcdInit = _est->getPosition();
    _pcd = _pcdInit;
    _RdInit = _lowState->getRotMat();

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
    firstrun = true;
    contact = VecInt4(1, 1, 1, 1);
}

void State_ThreefootStand::run(){
    _userValue = _lowState->userValue;
    // related
    _posFeetGlobal = _est->getFeetPos();

    pos_phase += (float)1 / pos_duration;
    pos_phase = pos_phase > 1 ? 1 : pos_phase;
    if (pos_phase >= 0.5)
    {
        action_phase += (float)1 / action_duration;
        action_phase = action_phase > 1 ? 1 : action_phase;
    }
    // related
    
    
    // related
    Vec3 leg_0_leg_3_center, desired_pos, desired_pcd;
    leg_0_leg_3_center = 0.5 * (_posFeetGlobal.col(0) + _posFeetGlobal.col(3));
    desired_pos = leg_0_leg_3_center + 0.25 * (_posFeetGlobal.col(1) - leg_0_leg_3_center);
    desired_pcd = _pcdInit + pos_phase * (desired_pos - _pcdInit); // 通过 pos_phase 来控制移动的缓慢。

    _pcd(0) = desired_pcd(0); // 将计算得到的期望重心的x坐标 赋值给_pcd，下行同理。
    _pcd(1) = desired_pcd(1);
    _pcd(2) = _pcdInit(2) - 0.05;
    // related

    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();
    _lowCmd->setStableGain();
    // related
    if (pos_phase >= 1.0)
    {
        contact(2) = 0;
        if (firstrun == true)
        {
            _qCurrent = vec12ToVec34(_q);
            firstrun = false;
        }
        _lowCmd->setSwingGain(2);
        _q(6) = _qCurrent.col(2)(0) + action_phase * (-0.9 - _qCurrent.col(2)(0));
        _q(7) = _qCurrent.col(2)(1) + action_phase * (2.46 - _qCurrent.col(2)(1));
        _q(8) = _qCurrent.col(2)(2) + action_phase * (-2.53 - _qCurrent.col(2)(2));
        _qdGoal.col(2) << 0, 0, 0;
        _tau(6) = 0;
        _tau(7) = 0;
        _tau(8) = 0;
    }
    // related

   
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_q);
}

void State_ThreefootStand::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    pos_phase = 0;
    pos_counter = 0;
    action_phase = 0;
}

FSMStateName State_ThreefootStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::THREEFOOTSTAND;
    }
}

void State_ThreefootStand::calcTau(){

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    // _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact); 
    _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, contact); // important! use fake contact schedule
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}