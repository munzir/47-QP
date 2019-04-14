// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/6/19
// Brief: Defining the QP for the upper body on Krang

#include "krang-qp/qp.hpp"

namespace qp {

// Define LeftArmOptParams
void qpLeftArmOptParams() {}
//void qpLeftArmOptParams(Eigen::Vector3d& _LeftTargetPosition, int numTaskDof, int numPassiveJoints, int numLowerBodyLinksOnBase, int numArmJoints, int numDof, int numBodyLinks, int mSteps, Eigen::MatrixXd mRot0, dart::dyanmics::BodyNode* mLeftEndEffector, Eigen::Vector3d mxyz0, bool mInverseKinematics, Eigen::Vector3d mdxyz0, ) {
//  static Eigen::Vector3d xEELref, xEEL, dxEEL, ddxEELref, dxref;
//  static Eigen::MatrixXd JEEL_small(
//      numTaskDof, numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints),
//      dJEEL_small(numTaskDof,
//                  numPassiveJoints + numLowerBodyLinksOnBase + numArmJoints);
//  static Eigen::MatrixXd JEEL_full(numTaskDof, numDof),
//      dJEEL_full(numTaskDof, numDof);
//  static Eigen::MatrixXd JEEL(numTaskDof, numBodyLinks),
//      dJEEL(numTaskDof, numBodyLinks);
//
//  xEELref = _LeftTargetPosition;
//  if (mSteps == 1) {
//    std::cout << "xEELref: " << xEELref(0) << ", " << xEELref(1) << ", "
//              << xEELref(2) << std::endl;
//  }
//
//  // x, dx, ddxref
//  xEEL = mRot0 * (mLeftEndEffector->getTransform().translation() - mxyz0);
//  if (!mInverseKinematicsOnArms) {
//    dxEEL = mRot0 * (mLeftEndEffector->getLinearVelocity() - mdxyz0) +
//            mdRot0 * (mLeftEndEffector->getTransform().translation() - mxyz0);
//    ddxEELref = -mKpEE * (xEEL - xEELref) - mKvEE * dxEEL;
//  } else {
//    dxref = -mKpEE * (xEEL - xEELref);
//  }
//
//  // Jacobian
//  JEEL_small = mLeftEndEffector->getLinearJacobian();
//  JEEL_full.setZero();
//  JEEL_full.leftCols(numPassiveJoints) = JEEL_small.leftCols(numPassiveJoints);
//  auto chain = getChainDofIndices(mLeftEndEffector);
//  for (int i = 1; i < chain.size(); i++)
//    JEEL_full.col(chain[i]) = JEEL_small.col(numPassiveJoints + i - 1);
//  JEEL = (mRot0 * JEEL_full * mJtf).topRightCorner(numTaskDof, numBodyLinks);
//
//  // Jacobian Derivative
//  if (!mInverseKinematicsOnArms) {
//    dJEEL_small = mLeftEndEffector->getLinearJacobianDeriv();
//    dJEEL_full.setZero();
//    dJEEL_full.leftCols(numPassiveJoints) =
//        dJEEL_small.leftCols(numPassiveJoints);
//    for (int i = 1; i < chain.size(); i++)
//      dJEEL_full.col(chain[i]) = dJEEL_small.col(numPassiveJoints + i - 1);
//    dJEEL = (mdRot0 * JEEL_full * mJtf + mRot0 * dJEEL_full * mJtf +
//             mRot0 * JEEL_full * mdJtf)
//                .topRightCorner(numTaskDof, numBodyLinks);
//
//    // P and b
//    mPEEL << mWEEL * JEEL;
//    mbEEL = -mWEEL * (dJEEL * mdqBody - ddxEELref);
//  } else {
//    mPEEL << mWEEL * JEEL;
//    mbEEL = mWEEL * dxref;
//  }
//}

// Define Right Arm Opt Params
void qpRightArmOptParams() {}

// Define Left Orientation Opt Params
void qpLeftOrientationOptParams() {}

// Define Right Orientation Opt Params
void qpRightOrientationOptParams() {}

// Define Balance Opt Params
void qpBalanceOptParams() {}

// Define Regulation Opt Params
void qpRegulationOptParams(bool mInverseKinematicsOnArms,
                           Eigen::MatrixXd mqBody, Eigen::MatrixXd mqBodyInit,
                           Eigen::MatrixXd mdqBody, Eigen::MatrixXd mWMatPose,
                           Eigen::MatrixXd mWMatSpeedReg,
                           Eigen::MatrixXd mWMatReg, double mKpPose,
                           double mKvPose, double mKvSpeedReg,
                           Eigen::MatrixXd& mPPose, Eigen::VectorXd& mbPose,
                           Eigen::MatrixXd& mPSpeedReg,
                           Eigen::VectorXd& mbSpeedReg, Eigen::MatrixXd& mPReg,
                           Eigen::VectorXd& mbReg) {
  if (!mInverseKinematicsOnArms) {
    mPPose = mWMatPose;
    mbPose << mWMatPose *
                  (-mKpPose * (mqBody - mqBodyInit) - mKvPose * mdqBody);

    mPSpeedReg = mWMatSpeedReg;
    mbSpeedReg << -mWMatSpeedReg * mKvSpeedReg * mdqBody;

    mPReg = mWMatReg;
    mbReg.setZero();
  } else {
    mPPose = mWMatPose;
    mbPose << mWMatPose * (-mKpPose * (mqBody - mqBodyInit));

    mPSpeedReg = mWMatSpeedReg;
    mbSpeedReg.setZero();

    mPReg = mWMatReg;
    mbReg = mWMatReg * mdqBody;
  }
}

// Define the P matrix in the QP
Eigen::MatrixXd qpdefineP(Eigen::MatrixXd mPEER, Eigen::MatrixXd mPOrR,
                          Eigen::MatrixXd mPEEL, Eigen::MatrixXd mPOrL,
                          Eigen::VectorXd mPBal, Eigen::MatrixXd mPPose,
                          Eigen::MatrixXd mPSpeedReg, Eigen::MatrixXd mPReg,
                          int mOptDim) {
  Eigen::MatrixXd P(mPEER.rows() + mPOrR.rows() + mPEEL.rows() + mPOrL.rows() +
                        mPBal.rows() + mPPose.rows() + mPSpeedReg.rows() +
                        mPReg.rows(),
                    mOptDim);
  P << mPEER.col(0), mPEER.topRightCorner(mPEER.rows(), mOptDim - 1),
      mPOrR.col(0), mPOrR.topRightCorner(mPOrR.rows(), mOptDim - 1),
      mPEEL.col(0), mPEEL.topRightCorner(mPEEL.rows(), mOptDim - 1),
      mPOrL.col(0), mPOrL.topRightCorner(mPOrL.rows(), mOptDim - 1),
      mPBal.col(0), mPBal.topRightCorner(mPBal.rows(), mOptDim - 1),
      mPPose.col(0), mPPose.topRightCorner(mPPose.rows(), mOptDim - 1),
      mPSpeedReg.col(0),
      mPSpeedReg.topRightCorner(mPSpeedReg.rows(), mOptDim - 1), mPReg.col(0),
      mPReg.topRightCorner(mPReg.rows(), mOptDim - 1);
  return P;
}

// Define the b matrix in the QP
Eigen::VectorXd qpdefineb(Eigen::MatrixXd mbEER, Eigen::MatrixXd mbOrR,
                          Eigen::MatrixXd mbEEL, Eigen::MatrixXd mbOrL,
                          Eigen::VectorXd mbBal, Eigen::MatrixXd mbPose,
                          Eigen::MatrixXd mbSpeedReg, Eigen::MatrixXd mbReg) {
  Eigen::VectorXd b(mbEER.rows() + mbOrR.rows() + mbEEL.rows() + mbOrL.rows() +
                        mbBal.rows() + mbPose.rows() + mbSpeedReg.rows() +
                        mbReg.rows(),
                    mbEER.cols());
  b << mbEER, mbOrR, mbEEL, mbOrL, mbBal, mbPose, mbSpeedReg, mbReg;
  return b;
}

}  // namespace qp
