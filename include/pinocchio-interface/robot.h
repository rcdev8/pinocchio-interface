// Pinocchio Robot Interface
// Rafael I. Cabral Muchacho

#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>

namespace pin = pinocchio;

class Robot {
  private:
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;

  public:
    Robot();
    Robot(const std::string &urdf_filename, const std::string &frame_ee_name, const Eigen::VectorXd &q, const Eigen::VectorXd &dq);

    pin::Model model;
    pin::DataTpl<double, 0, pin::JointCollectionDefaultTpl> data;

    pin::FrameIndex frame_ee_idx;
    pin::JointIndex joint_ee_idx;

    // kinematics
    Eigen::MatrixXd J; // zero jacobian
    Eigen::MatrixXd O_T_EE;
    Eigen::VectorXd O_dP_EE;

    void updateKinematics(Eigen::VectorXd q, Eigen::VectorXd dq);

    // dynamics
    Eigen::MatrixXd mass;
    Eigen::MatrixXd invMass;
    Eigen::MatrixXd coriolis;
    Eigen::VectorXd gravity;

    void updateDynamics(Eigen::VectorXd q, Eigen::VectorXd dq);
    void updateDynamics(); 
    
    Eigen::MatrixXd getCoriolis(Eigen::VectorXd q, Eigen::VectorXd dq);
    Eigen::MatrixXd getdJ(Eigen::VectorXd q, Eigen::VectorXd dq); // zero jacobian time variation 

    Eigen::VectorXd getq(){return q_;}
    Eigen::VectorXd getdq(){return dq_;}

};


