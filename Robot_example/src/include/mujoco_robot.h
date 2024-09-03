#ifndef WEBOTS_ROBOT_HPP_
#define WEBOTS_ROBOT_HPP_
#include <mujoco/mujoco.h>
#include <unistd.h>

class MujocoRobotImpl
{
public:
  MujocoRobotImpl(mjModel *model, mjData *data);
  ~MujocoRobotImpl(){};
  void Run();
  mjData *mj_data_;
  mjModel *mj_model_;
  int cmd_ = 0;
private:
};

#endif // WEBOTS_ROBOT_HPP_
