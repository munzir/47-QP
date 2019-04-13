// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse kinematics

#ifndef KRANG_QP__IK_HPP
#define KRANG_QP__IK_HPP

#include <dart/dart.hpp>
#include <nlopt.hpp>

// See comment on id.hpp
#include "krang-qp/id.hpp"

//namespace ik {

//struct OptParams {
//  Eigen::MatrixXd P;
//  Eigen::VectorXd b;
//};

// the same thing happens for the optFunc definition below
// // opt function
// double optFunc(const std::vector<double>& x, std::vector<double>& grad,
//              void* my_func_data);

// Function Prototypes

// // Define Regulation Opt Params P
// Eigen::MatrixXd definePReg(mWMatPose, mWMatSpeedReg,);

// // Define Regulation Opt Params b
// Eigen::MatrixXd definePReg();

// // Define P for QP
// Eigen::MatrixXd defineP(Eigen::MatrixXd mPEER,
//                        Eigen::MatrixXd mPOrR,
//                        Eigen::MatrixXd mPEEL,
//                        Eigen::MatrixXd mPOrL,
//                        Eigen::VectorXd mPBal,
//                        Eigen::MatrixXd mPPose,
//                        Eigen::MatrixXd mPSpeedReg,
//                        Eigen::MatrixXd mPReg, int mOptDim);
//
//// // Define b for QP
// Eigen::VectorXd defineb(Eigen::MatrixXd mbEER,
//                        Eigen::MatrixXd mbOrR,
//                        Eigen::MatrixXd mbEEL,
//                        Eigen::MatrixXd mbOrL,
//                        Eigen::VectorXd mbBal,
//                        Eigen::MatrixXd mbPose,
//                        Eigen::MatrixXd mbSpeedReg,
//                        Eigen::MatrixXd mbReg);

// // compute speeds for joints based on ik algorithm
Eigen::VectorXd computeSpeeds(int mOptDim, OptParams optParams, bool maxTimeSet,
                              Eigen::VectorXd mdqBodyRef);

//}  // namespace ik
#endif  // KRANG_QP__IK_HPP
