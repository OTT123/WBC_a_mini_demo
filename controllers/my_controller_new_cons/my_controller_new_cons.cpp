// File:          my_controller_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <qpOASES.hpp>
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
#define DOF 4
// All the webots classes are defined in the "webots" namespace
Eigen::MatrixXd Jacobian(Eigen::VectorXd pos, Eigen::VectorXd len)
{
  double q1 = pos(0);
  double q2 = pos(1);
  double q3 = pos(2);
  double q4 = pos(3);
  double L1 = len(0);
  double L2 = len(1);
  double L3 = len(2);
  double L4 = len(3);
  Eigen::MatrixXd Jac = Eigen::MatrixXd(3, DOF);
  Jac(0, 0) = L1 * cos(q1) + L2 * cos(q1 + q2) + L3 * cos(q1 + q2 + q3) + L4 * cos(q1 + q2 + q3 + q4);
  Jac(0, 1) = L2 * cos(q1 + q2) + L3 * cos(q1 + q2 + q3) + L4 * cos(q1 + q2 + q3 + q4);
  Jac(0, 2) = L3 * cos(q1 + q2 + q3) + L4 * cos(q1 + q2 + q3 + q4);
  Jac(0, 3) = L4 * cos(q1 + q2 + q3 + q4);

  Jac(1, 0) = -L1 * sin(q1) - L2 * sin(q1 + q2) - L3 * sin(q1 + q2 + q3) - L4 * sin(q1 + q2 + q3 + q4);
  Jac(1, 1) = -L2 * sin(q1 + q2) - L3 * sin(q1 + q2 + q3) - L4 * sin(q1 + q2 + q3 + q4);
  Jac(1, 2) = -L3 * sin(q1 + q2 + q3) - L4 * sin(q1 + q2 + q3 + q4);
  Jac(1, 3) = -L4 * sin(q1 + q2 + q3 + q4);

  Jac(2, 0) = 1;
  Jac(2, 1) = 1;
  Jac(2, 2) = 1;
  Jac(2, 3) = 1;
  return Jac;
}

Eigen::Vector3d fd_kin(Eigen::VectorXd pos, Eigen::VectorXd len)
{
  double q1 = pos(0);
  double q2 = pos(1);
  double q3 = pos(2);
  double q4 = pos(3);
  double L1 = len(0);
  double L2 = len(1);
  double L3 = len(2);
  double L4 = len(3);
  Eigen::VectorXd pos_end(DOF);
  pos_end(0) = L1 * sin(q1) + L2 * sin(q1 + q2) + L3 * sin(q1 + q2 + q3) + L4 * sin(q1 + q2 + q3 + q4);
  pos_end(1) = L1 * cos(q1) + L2 * cos(q1 + q2) + L3 * cos(q1 + q2 + q3) + L4 * cos(q1 + q2 + q3 + q4);
  pos_end(2) = q1 + q2 + q3 + q4;
  return pos_end;
}

using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  std::string joint_name[DOF] = {{"joint_1"}, {"joint_2"}, {"joint_3"}, {"joint_4"}};
  webots::Motor *motor[DOF];
  webots::PositionSensor *pos_sensor[DOF];
  

  for (int i = 0; i < DOF; ++i)
  {
    motor[i] = robot->getMotor(joint_name[i]);
    motor[i]->setPosition(0.1);
    pos_sensor[i] = robot->getPositionSensor(joint_name[i] + "_sensor");
    pos_sensor[i]->enable(timeStep);
  }
  
  
  int time_iter = 0;
  Eigen::VectorXd Len(DOF);
  for (int i = 0; i < DOF; ++i)
  {
    Len(i) = 1.0;
  }

  // 目标位置，相对于基坐标系
  Eigen::Vector3d pos_target;
  // 正常target
  // 有grad，无振荡
  // 无grad，无振荡
  // pos_target << 1.41, 3.41, 0.2;

  // 雅可比奇异target
  // 该target无论是否有grad，速度指令都有轻微振荡
  // pos_target << 1.0, 4.0, 0;

  // 关节限位target
  // 无论有无grad，都几乎没有振荡
  // pos_target << 4.0, 1.0, 0;

  // 雅可比奇异target
  // 该target无论是否有grad，速度指令都有剧烈振荡
  // pos_target << 0.0, 5.0, 0;

  // 雅可比奇异target
  // 如果没有grad，则不动
  // 如果有，则不会稳定到零位22
  // pos_target << 0.0, 4.0, 0;

  // testtarget
  // 无论有无grad，都几乎没有振荡
  pos_target << 0.71, 3.7, 0;
  double KP = 1;
  double KD = 1;

  // 关节速度限制
  Eigen::VectorXd up_vel_limit(DOF);
  up_vel_limit << 0.5, 0.5, 0.5, 0.5;
  Eigen::VectorXd low_vel_limit(DOF);
  low_vel_limit << -0.5, -0.5, -0.5, -0.5;
  double threshold = 0.1;

  // 关节位置限制
  Eigen::VectorXd up_pos_limit(DOF);
  Eigen::VectorXd low_pos_limit(DOF);
  up_pos_limit << 0.5, 0.5, 0.5, 0.5;
  low_pos_limit << -0.5, -0.5, -0.5, -0.5;
  
  bool init = false;
  Eigen::VectorXd pre_pos(DOF);
  pre_pos.setZero();
  // Main loop:
  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    // double val = ds->getValue();
    double time = double(time_iter) * timeStep / 1000;
    std::cout << "************ time= " << time << " ************" << std::endl;
    // getJointJacobian(LOCAL_WORLD_ALIGNED)
    
    if(!init){
      for (int i = 0; i < DOF; ++i)
      {
        pre_pos(i) = pos_sensor[i]->getValue();
      }
      init = true;
    }
    
    // calc real position and velocity
    Eigen::VectorXd pos_real(DOF);
    for (int i = 0; i < DOF; ++i)
    {
      pos_real(i) = pos_sensor[i]->getValue();
    }
    Eigen::VectorXd vec_real(DOF);

    vec_real = (pos_real - pre_pos) / (double(timeStep) / 1000);
    pre_pos = pos_real;
    std::cout << "pos_real= " << pos_real.transpose() << std::endl;
    std::cout << "vec_real= " << vec_real.transpose() << std::endl;

    auto Jac = Jacobian(pos_real, Len);
    int num_var = 2 * DOF + 3;
    int num_output = 3; // 平面问题，速度只有3个变量


    // Hess
    Eigen::MatrixXd Hess = Eigen::MatrixXd(num_var, num_var);
    Hess.block(0, 0, DOF, DOF).diagonal() << 20, 20, 20, 20;
    Hess.block(DOF, DOF, num_output, num_output).diagonal() << 1e4, 1e4, 1e4;
    Hess.block(DOF + num_output, DOF + num_output, DOF, DOF).diagonal() << 0,0,0,0;


    //grad
    Eigen::VectorXd grad(num_var);
    grad.setZero();


    //lbx, ubx
    Eigen::VectorXd lbx(num_var);
    Eigen::VectorXd ubx(num_var);
    lbx.head(DOF) = low_vel_limit;
    lbx.segment(DOF, num_output) << -1e6, -1e6, -1e6;
    lbx.tail(DOF) << -1e6, -1e6, -1e6,-1e6;
    ubx.head(DOF) = up_vel_limit;
    ubx.segment(DOF, num_output) << 1e6, 1e6, 1e6;
    ubx.tail(DOF) << 1e6, 1e6, 1e6, 1e6;
    // 根据关节位置处理速度限制
    for (int i = 0; i < DOF; ++i)
    {
      double temp_up = up_pos_limit(i) - pos_real(i);
      ubx(i) = temp_up > threshold ? up_vel_limit(i) : (temp_up / threshold) * up_vel_limit(i);
      if (temp_up < 0)
      {
        ubx(i) = 0.0;
      }
      double temp_low = pos_real(i) - low_pos_limit(i);
      lbx(i) = temp_low > threshold ? low_vel_limit(i) : (temp_low / threshold) * low_vel_limit(i);
      if (temp_low < 0)
      {
        lbx(i) = 0.0;
      }
    }


    // CST and lbc ubc
    Eigen::MatrixXd Cst = Eigen::MatrixXd(num_output + DOF, num_var);
    Cst.setZero();
    Cst.block(0, 0, num_output, DOF) = Jac;
    Cst.block(0, DOF, num_output, num_output).setIdentity();
    Cst.block(num_output, 0, DOF, DOF).setIdentity();
    Cst.block(num_output, num_output + DOF, DOF, DOF).setIdentity();

    double EPS = 1e-12;

    Eigen::VectorXd lbc(DOF + num_output);
    Eigen::VectorXd ubc(DOF + num_output);

    Eigen::Vector3d vec_real_end;
    vec_real_end = Jac * vec_real;            // 真实末端坐标系速度旋量
    auto pos_end = fd_kin(pos_real, Len);     // 真实末端坐标系位置
    Eigen::Vector3d vec_target;               // 末端坐标系目标速度旋量
    vec_target = (pos_target - pos_end) * KP; // 比例控制

    // vec_target << 0,1,0;
    lbc.head(num_output) = vec_target.array() - EPS;
    ubc.head(num_output) = vec_target.array() + EPS;
    lbc.tail(DOF) = vec_real;
    ubc.tail(DOF) = vec_real;

    Eigen::VectorXd res(num_var);
    res.setZero();

    qpOASES::QProblem qp(num_var, num_output + DOF);
    qpOASES::Options options;
    // options.setToReliable();
    options.setToMPC();
    options.terminationTolerance = 1e-6;
    // options.printLevel = qpOASES::PL_HIGH;
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);
    int nWSR = 1e4;
    MatrixRowMajor Hess_row(Hess);
    MatrixRowMajor Cst_row(Cst);
    int init_qp = qp.init(Hess_row.data(), grad.data(), Cst_row.data(), lbx.data(), ubx.data(), lbc.data(), ubc.data(), nWSR);
    if (init_qp != qpOASES::SUCCESSFUL_RETURN)
    {
      std::cout << "Failed solve qp" << std::endl;
    }
    else
    {
      qp.getPrimalSolution(res.data());
    }

    
    std::cout << "real end pos = [" << pos_end.transpose() << "];" << std::endl;
    std::cout << "target end velocity screw = [" << vec_target.transpose() << "];"  << std::endl;
    std::cout << "real end velocity screw = [" << vec_real_end.transpose() << "];"  << std::endl;
    // std::cout << "Hess = [\n" << Hess << "];" << std::endl;
    // std::cout << "grad = [" << grad.transpose() << "];" << std::endl;
  
    std::cout << "Cst = [\n"
              << Cst << "];" << std::endl;
    std::cout << "lbx = [" << lbx.transpose() << "];" << std::endl;
    std::cout << "ubx = [" << ubx.transpose() << "];" << std::endl;
    std::cout << "lbc = [" << lbc.transpose() << "];" << std::endl;
    std::cout << "ubc = [" << ubc.transpose() << "];" << std::endl;
    std::cout << "opt_value = [" << res.transpose() << "];" << std::endl;
    std::cout << "Cst * Opt = " << (Cst * res).transpose() << std::endl;
    std::cout<<"Jacobian * dq =  [";
    std::cout<<(Jac * res.head(DOF)).transpose()<<" ];"<<std::endl;
    
    std::cout << "mt = " << std::sqrt((Jac * Jac.transpose()).determinant()) << std::endl;

    // 发布速度指令
    for (int i = 0; i < DOF; ++i)
    {
      if (pos_real(i) > up_pos_limit(i) - 1e-6)
      {
        // res(i) = -1e-3;
        res(i) = 0.0;
      }
      if (pos_real(i) < low_pos_limit(i) + 1e-6)
      {
        // res(i) = -1e-3;
        res(i) = 0.0;
      }
      motor[i]->setPosition(INFINITY);
      motor[i]->setVelocity(res(i));
    }

    time_iter++;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
