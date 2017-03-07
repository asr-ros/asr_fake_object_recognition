/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "rating.h"
#include <Eigen/Dense>
#include <ros/console.h>

namespace fake_object_recognition {

Rating::Rating(double fovx, double fovy, double ncp, double fcp) :
    fovx_(fovx),
    fovy_(fovy),
    ncp_(ncp),
    fcp_(fcp),
    camera_orientation_(Eigen::Vector3d(0.0, 0.0, 1.0)) { }

bool Rating::ratePose(const geometry_msgs::Pose &pose, double threshold_d, double threshold_x, double threshold_y) {

    double rating_d = getDistanceRating(pose);
    ROS_DEBUG_STREAM("Distance Rating: " << rating_d);
    double rating_x = getAngleRating(pose, true);
    ROS_DEBUG_STREAM("Angle rating in xz-plane: " << rating_x);
    double rating_y = getAngleRating(pose, false);
    ROS_DEBUG_STREAM("Angle rating in yz-plane: " << rating_y);

    return rating_d > threshold_d && rating_x > threshold_x && rating_y > threshold_y;
}

double Rating::getDistanceRating(const geometry_msgs::Pose &object_pose) {
    double rating = 0.0;
    Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
    double dot_cam_obj = camera_orientation_.dot(object_position);

    double dist_to_mid = std::abs(dot_cam_obj - ((fcp_ + ncp_) / 2.0));
    double dist_threshold = (fcp_ - ncp_) / 2.0;

    if (dist_to_mid < dist_threshold) {
        rating = 0.5 + 0.5 * cos((dist_to_mid * M_PI) / dist_threshold);
    }

    return rating;
}

  double Rating::getAngleRating(const geometry_msgs::Pose &object_pose, bool direction) {
    double rating = 0.0;
    
    Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);

    double angle_max;
    
    //Azimut
    if(direction){
      //Project object position on xz-plane of camera frame.
      object_position.y() = 0.0;
      angle_max = (fovx_ * M_PI) / 180.0;
    }
    //Elevation
    else {
      //Project object position on xz-plane of camera frame.
      object_position.x() = 0.0;     
      angle_max = (fovy_ * M_PI) / 180.0;
    }

    object_position.normalize();

    double angle = std::acos(camera_orientation_.dot(object_position));

    if (angle < angle_max) {
        rating = 0.5 + 0.5 * std::cos((angle * M_PI) / angle_max);  
    }

    return rating;
}

}
