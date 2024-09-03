#include "include/mujoco_robot.h"
#include<iostream>
MujocoRobotImpl::MujocoRobotImpl(mjModel *model, mjData *data):mj_model_(model), mj_data_(data){
}

void MujocoRobotImpl::Run()
{
    while (1)
    {
      // mj_data_->ctrl[0] = cmd_;
      // cmd_ += 1;
      std::cout << "leg pos is:" << mj_data_->sensordata[0] << std::endl;

      sleep(2);
    }
}
