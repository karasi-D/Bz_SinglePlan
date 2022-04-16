#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include "mosek.h"
#include "bezier_base.h"
#include "data_type.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

using namespace std;
using namespace Eigen;

class TrajectoryGenerator {
private:
        // int _traj_order;
        MatrixXd _bezier_coeff;
public:
        TrajectoryGenerator(){}
        ~TrajectoryGenerator(){}

        /* Use Bezier curve for the trajectory */
       int BezierPloyCoeffGeneration(
            const vector<Cube> &corridor,
            const MatrixXd &MQM,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            const int traj_order,
            const double minimize_order,
            const double margin,
            const bool & isLimitVel,
            const bool & isLimitAcc,
            double & obj,
            MatrixXd & PolyCoeff); 

        quadrotor_msgs::PolynomialTrajectory getBezierTraj(const MatrixXd & polyCoeff, const VectorXd & _seg_time, int _traj_order, int _seg_num, int _traj_id, ros::Time t_now );
        Vector3d getPosFromBezier(const MatrixXd & polyCoeff, const VectorXd & _C,  int _traj_order, double t_now, int seg_now );
        Vector3d getVelFromBezier(const MatrixXd & polyCoeff, const VectorXd & _Cv, int _traj_order, double t_now, int seg_now);
        VectorXd getStateFromBezier(const MatrixXd & polyCoeff, const VectorXd & _C, const VectorXd & _Cv, const VectorXd & _Ca, const VectorXd & _Cj, int _traj_order, double t_now, int seg_now );
        
};

#endif