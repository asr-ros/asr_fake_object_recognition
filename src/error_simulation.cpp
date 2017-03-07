/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "error_simulation.h"
#include <random>
#include <iostream>
#include <ros/console.h>
#include <Eigen/Dense>


namespace fake_object_recognition {

ErrorSimulation::ErrorSimulation() : ErrorSimulation(false, false, false) { }

ErrorSimulation::ErrorSimulation(bool use_pose_invalidation, bool use_position_noise, bool use_orientation_noise) :
    use_pose_invalidation_(use_pose_invalidation),
    use_position_noise_(use_position_noise),
    use_orientation_noise_(use_orientation_noise),
    prob_pose_inval_(DEFAULT_PROB_POSE_INVAL),
    pos_noise_dist_mean_(DEFAULT_POS_NOISE_MEAN),
    pos_noise_dist_dev_(DEFAULT_POS_NOISE_DEV),
    or_x_noise_dist_mean_(DEFAULT_OR_X_NOISE_MEAN),
    or_x_noise_dist_dev_(DEFAULT_OR_X_NOISE_DEV),
    or_y_noise_dist_mean_(DEFAULT_OR_Y_NOISE_MEAN),
    or_y_noise_dist_dev_(DEFAULT_OR_Y_NOISE_DEV),
    or_z_noise_dist_mean_(DEFAULT_OR_Z_NOISE_MEAN),
    or_z_noise_dist_dev_(DEFAULT_OR_Z_NOISE_DEV) { }

double ErrorSimulation::genUniformNumber(double interval_start, double interval_end) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(interval_start, interval_end);
    return dis(gen);
}

double ErrorSimulation::genNormalDistNumber(double mean, double std_deviation) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dis(mean, std_deviation);
    return dis(gen);
}

bool ErrorSimulation::poseInvalidation() {
    if (use_pose_invalidation_) {
        return genUniformNumber(0.0, 1.0) >= prob_pose_inval_;
    }
    return true;
}

geometry_msgs::Pose ErrorSimulation::addNoiseToPosition(const geometry_msgs::Pose &pose) {
    if (use_position_noise_) {
        geometry_msgs::Pose result(pose);
        result.position.x += genNormalDistNumber(pos_noise_dist_mean_, pos_noise_dist_dev_);
        result.position.y += genNormalDistNumber(pos_noise_dist_mean_, pos_noise_dist_dev_);
        result.position.z += genNormalDistNumber(pos_noise_dist_mean_, pos_noise_dist_dev_);
        return result;
    }
    return pose;
}

geometry_msgs::Pose ErrorSimulation::addNoiseToOrientation(const geometry_msgs::Pose &pose) {
    if (use_orientation_noise_) {
        geometry_msgs::Pose result(pose);
        Eigen::Quaternion<double> orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Quaternion<double> quat_x(Eigen::AngleAxis<double>(genNormalDistNumber(or_x_noise_dist_mean_, or_x_noise_dist_dev_), Eigen::Vector3d(1.0, 0.0, 0.0)));
        Eigen::Quaternion<double> quat_y(Eigen::AngleAxis<double>(genNormalDistNumber(or_y_noise_dist_mean_, or_y_noise_dist_dev_), Eigen::Vector3d(0.0, 1.0, 0.0)));
        Eigen::Quaternion<double> quat_z(Eigen::AngleAxis<double>(genNormalDistNumber(or_z_noise_dist_mean_, or_z_noise_dist_dev_), Eigen::Vector3d(0.0, 0.0, 1.0)));

        orientation = quat_x * quat_y * quat_z * orientation;
        result.orientation.w = orientation.w();
        result.orientation.x = orientation.x();
        result.orientation.y = orientation.y();
        result.orientation.z = orientation.z();
        return result;
    }
    return pose;
}


void ErrorSimulation::setProbPoseInval(double prob_pose_inval) {
    prob_pose_inval_ = prob_pose_inval;
}
void ErrorSimulation::setPoseNoiseDistMean(double pos_noise_dist_mean) {
    pos_noise_dist_mean_ = pos_noise_dist_mean;
}

void ErrorSimulation::setPoseNoiseDistDev(double pos_noise_dist_dev) {
    pos_noise_dist_dev_ = pos_noise_dist_dev;
}

void ErrorSimulation::setOrXNoiseDistMean(double or_x_noise_dist_mean) {
    or_x_noise_dist_mean_ = or_x_noise_dist_mean;
}

void ErrorSimulation::setOrXNoiseDistDev(double or_x_noise_dist_dev) {
    or_x_noise_dist_dev_ = or_x_noise_dist_dev;
}

void ErrorSimulation::setOrYNoiseDistMean(double or_y_noise_dist_mean) {
    or_y_noise_dist_mean_ = or_y_noise_dist_mean;
}

void ErrorSimulation::setOrYNoiseDistDev(double or_y_noise_dist_dev) {
    or_y_noise_dist_dev_ = or_y_noise_dist_dev;
}

void ErrorSimulation::setOrZNoiseDistMean(double or_z_noise_dist_mean) {
    or_z_noise_dist_mean_ = or_z_noise_dist_mean;
}

void ErrorSimulation::setOrZNoiseDistDev(double or_z_noise_dist_dev) {
    or_z_noise_dist_dev_ = or_z_noise_dist_dev;
}

}
