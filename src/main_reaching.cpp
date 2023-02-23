#include "tsid_manipulator_reaching.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iostream>
#include <fstream>

namespace pin = pinocchio;

int main()
{
    std::string ee_frame_pin = "panda_link8";

    pin::Model model_pin;
    std::string model_path = "/home/mfourmy/catkin_ws/src/panda_torque_mpc/config/panda_inertias_nohand.urdf";
    pin::urdf::buildModel(model_path, model_pin);
    pin::Data data_pin(model_pin);

    TsidConfig conf;
    conf.q0 = (Vector7d()<< 0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397).finished();
    TsidManipulatorReaching tsid_reaching_(model_path, conf);

    tsid_reaching_.setPostureRef(conf.q0);

    pin::forwardKinematics(model_pin, data_pin, conf.q0);
    pin::updateFramePlacements(model_pin, data_pin);
    pin::SE3 T_ee0 = data_pin.oMf[model_pin.getFrameId(ee_frame_pin)];
    pin::Motion dx_r = pin::Motion::Zero(); 
    pin::Motion ddx_r = pin::Motion::Zero(); 
    Eigen::Vector3d offset; offset << 0.3, 0.2, 0.0;

    pin::SE3 x_r = T_ee0;
    x_r.translation() += offset;

    // loop variables
    int N = 3000;
    double dt = 1e-3;
    
    Vector7d q = conf.q0;
    Vector7d v = Vector7d::Zero();
    std::ofstream file_q;
    std::ofstream file_v;
    std::ofstream file_tau;
    std::ofstream file_T;
    file_q.open("tsid_out_q.csv");
    file_v.open("tsid_out_v.csv");
    file_tau.open("tsid_out_tau.csv");
    file_T.open("tsid_out_T.csv");

    file_q << "q0,q1,q2,q3,q4,q5,q6" << "\n";
    file_v << "v0,v1,v2,v3,v4,v5,v6" << "\n";
    file_tau << "tau0,tau1,tau2,tau3,tau4,tau5,tau6" << "\n";
    file_T << "tx,ty,tz,ox,oy,oz,tx_r,ty_r,tz_r,ox_r,oy_r,oz_r" << "\n";

    for (int i=0; i < N; i++)
    {        
        tsid_reaching_.solve(q, v);
        tsid_reaching_.setEERef(x_r, dx_r, ddx_r);
        Eigen::VectorXd dv = tsid_reaching_.getAccelerations();
        Eigen::VectorXd tau_d = tsid_reaching_.getTorques();

        pin::forwardKinematics(model_pin, data_pin, q);
        pin::updateFramePlacements(model_pin, data_pin);
        pin::SE3 T_ee = data_pin.oMf[model_pin.getFrameId(ee_frame_pin)];

        file_q << q(0) << ","
               << q(1) << ","
               << q(2) << ","
               << q(3) << ","
               << q(4) << ","
               << q(5) << ","
               << q(6) << "\n";

        file_v << v(0) << ","
               << v(1) << ","
               << v(2) << ","
               << v(3) << "," 
               << v(4) << ","
               << v(5) << ","
               << v(6) << "\n";

        file_tau << tau_d(0) << ","
                 << tau_d(1) << ","
                 << tau_d(2) << ","
                 << tau_d(3) << ","
                 << tau_d(4) << ","
                 << tau_d(5) << ","
                 << tau_d(6) << "\n";

        Eigen::Vector3d aa_ee = pin::log3(T_ee.rotation());
        Eigen::Vector3d aa_r = pin::log3(x_r.rotation());
        
        file_T << T_ee.translation()(0) << ","
               << T_ee.translation()(1) << ","
               << T_ee.translation()(2) << ","
               << aa_ee(0) << ","
               << aa_ee(1) << ","
               << aa_ee(2) << ","
               << x_r.translation()(0) << ","
               << x_r.translation()(1) << ","
               << x_r.translation()(2) << ","
               << aa_r(0) << ","
               << aa_r(1) << ","
               << aa_r(2) << "\n";

        tsid_reaching_.integrate_dv(q, v, dv, dt);


    } 

    file_q.close();
    file_v.close();
    file_tau.close();
    file_T.close();
    
}