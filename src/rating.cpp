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

#include <pcl/point_cloud.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/filters/impl/frustum_culling.hpp>

namespace fake_object_recognition {

Rating::Rating(double fovx, double fovy, double ncp, double fcp) :
    fovx_(fovx),
    fovy_(fovy),
    ncp_(ncp),
    fcp_(fcp),
    camera_orientation_(Eigen::Vector3d(0.0, 0.0, 1.0)) { }

bool Rating::ratePose(const geometry_msgs::Pose &pose, double threshold_d, double threshold_x, double threshold_y) {
    //Use distance and agle ratings to rate pose.
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

bool Rating::getBBRating(const std::array<geometry_msgs::Point, 8> &bounding_box) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    unsigned int bb_size = bounding_box.size(); // always 8
    for (unsigned int i = 0; i < bb_size; i++) {
        pcl::PointXYZ pt = pcl::PointXYZ(bounding_box.at(i).x, bounding_box.at(i).y, bounding_box.at(i).z);
        point_cloud->push_back(pt);
    }

    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setHorizontalFOV(fovx_);
    fc.setVerticalFOV(fovy_);
    fc.setNearPlaneDistance(ncp_);
    fc.setFarPlaneDistance(fcp_);

    /* Camera pose: since visual axis is always set to (0,0,1) in ctor, a fixed matrix is used here; it is a homogenous matrix rotating around the y-axis by 90 degrees
     * because of the assumption that the camera is pointing in the x direction in the beginning. */
    Eigen::Matrix4f cam;
    cam <<  0.0, 0.0, -1.0, 0.0,
            0.0, 1.0,  0.0, 0.0,
            1.0, 0.0,  0.0, 0.0,
            0.0, 0.0,  0.0, 1.0;
    fc.setCameraPose(cam);

    fc.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(point_cloud));

    pcl::PointCloud<pcl::PointXYZ>::Ptr culled_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    fc.filter(*culled_cloud);

    return (culled_cloud->size() == bb_size);
}

double Rating::getNormalRating(const geometry_msgs::Pose &object_pose, const std::vector<geometry_msgs::Point> &normals) {
    //similar to NBV - DefaulRatingModule
    // (never called with empty normals because of check in rateBBandNormal)
    Eigen::Vector3d v1, v2;
    double angleThreshold = M_PI * 0.5;
    v1 << object_pose.position.x, object_pose.position.y, object_pose.position.z; // Vector describing the position of the object.
    v1 = (-1) * v1;
    v1.normalize(); // Vector from camera to object

    float xNorm = v1.lpNorm<2>();
    if (xNorm == 0) {
        ROS_DEBUG_STREAM("Object appears to be inside camera.");
        return 0.0;
    }
    // find maximum rating:
    float best_rated = 0.0;
    for (unsigned int i = 0; i < normals.size(); i++) {
        v2 << normals.at(i).x, normals.at(i).y, normals.at(i).z;
        v2.normalize();
        float yNorm = v2.lpNorm<2>();
        if (yNorm != 0) { // Otherwise normal does not actually exist
            float cosine = v1.dot(v2) / xNorm / yNorm;
            float angle = std::acos(cosine);
            if (angle < angleThreshold) {
                float rating = 0.5 + 0.5 * std::cos(angle * M_PI / angleThreshold);
                if (rating > best_rated) { best_rated = rating; }
            }           
        }
    }
    ROS_DEBUG_STREAM("getNormalRating: best_rated = " << best_rated);
    return best_rated;
}

bool Rating::rateBBandNormal(const geometry_msgs::Pose &object_pose, const std::array<geometry_msgs::Point, 8> &bounding_box, const std::vector<geometry_msgs::Point> &normals, double threshold) {
    if (!getBBRating(bounding_box)) {       // if not all 8 corner points of bounding box are inside the frustum:
        return false;                       // Return immediately.
    }
    else if (normals.empty()) {             // if there are no normals, e.g. because an object has no clear ones:
        return true;                        // since all bounding box corner points are inside frustum: return true.
    }
    else {
        return (getNormalRating(object_pose, normals) > threshold); // Otherwise rate normals.
    }
}




}
