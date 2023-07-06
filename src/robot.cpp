// Pinocchio Robot Interface
// Rafael I. Cabral Muchacho

#include <pinocchio-interface/robot.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp> // coriolis, gravity
#include <pinocchio/algorithm/aba.hpp> // inverse inertia
#include <pinocchio/algorithm/crba.hpp> // inertia

namespace pin = pinocchio;

Robot::Robot(){}

Robot::Robot(const std::string &urdf_filename, const std::string &frame_ee_name, const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {

    pin::urdf::buildModel(urdf_filename, model);
    data = pin::Data(model);

    // resize variables to fit model
    q_.resize(model.nv);
    dq_.resize(model.nv);
    J.resize(6, model.nv);
    O_T_EE.resize(4, 4);
    O_dP_EE.resize(6);

    mass.resize(model.nv, model.nv);
    invMass.resize(model.nv, model.nv);
    coriolis.resize(model.nv, model.nv);
    gravity.resize(6);

    // print model structure
    std::cout << "---- joint names: " << std::endl;
    for (auto name : model.names){
      std::cout << name << std::endl; 
    }

    std::cout << "---- frame names: " << std::endl;
    for (auto frame : model.frames){
      std::cout << frame.name << std::endl; 
    }

    // set end effector frame
    if (model.existFrame(frame_ee_name, pin::FrameType::BODY)){
        frame_ee_idx = model.getFrameId(frame_ee_name);
        std::cout << "ee frame id: " << frame_ee_idx << std::endl;
    } else {
        throw std::invalid_argument( "input end effector frame \"" + frame_ee_name + "\" does not exist");
    }

    // set last joint id
    joint_ee_idx = model.getJointId(model.names.back());
    std::cout << "joint ee frame id: " << joint_ee_idx << std::endl;

    updateKinematics(q, dq);
    updateDynamics(q, dq);

    std::cout << "created " << model.name << " model" << std::endl;

    return;
}

void Robot::updateKinematics(Eigen::VectorXd q, Eigen::VectorXd dq) {
  q_ = q;
  dq_ = dq;

  pin::forwardKinematics(model, data, q_, dq_);
  pin::computeJointJacobians(model, data, q_);
  pin::updateFramePlacements(model, data);

  J.setZero();
  pin::getFrameJacobian(model, data, frame_ee_idx, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  
  O_T_EE = data.oMf[frame_ee_idx].toHomogeneousMatrix();
  O_dP_EE = pin::getFrameVelocity(model, data, frame_ee_idx, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector();
  
  return;
}

void Robot::updateDynamics(){
  updateDynamics(q_, dq_);
  return;
}

void Robot::updateDynamics(Eigen::VectorXd q, Eigen::VectorXd dq){
  // does not set internal joint state (q, dq) object attributes
  coriolis = pin::computeCoriolisMatrix(model, data, q, dq);

  invMass = pin::computeMinverse(model, data, q); // returns only upper triangular
  invMass.triangularView<Eigen::StrictlyLower>() = invMass.transpose().triangularView<Eigen::StrictlyLower>();

  mass = pin::crba(model, data, q);
  mass.triangularView<Eigen::StrictlyLower>() = mass.transpose().triangularView<Eigen::StrictlyLower>();

  gravity = pin::computeGeneralizedGravity(model, data, q);

  return;
}


Eigen::MatrixXd Robot::getCoriolis(Eigen::VectorXd q, Eigen::VectorXd dq){
  // does not update coriolis attribute
  return pin::computeCoriolisMatrix(model, data, q, dq);
}

Eigen::MatrixXd Robot::getdJ(Eigen::VectorXd q, Eigen::VectorXd dq){
  Eigen::MatrixXd dJ(6, model.nv);
  dJ.setZero();
  pin::computeJointJacobiansTimeVariation(model, data, q, dq);
  pin::getFrameJacobianTimeVariation(model, data, frame_ee_idx, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
  return dJ;
}