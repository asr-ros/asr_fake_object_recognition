/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RATING_H
#define RATING_H

#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

namespace fake_object_recognition {

/**
 * \brief This class is used to rate an object's pose based on the current camera position
 */
class Rating {

    /** The field of view in x-direction of the camera **/
    double fovx_;

    /** The field of view in y-direction of the camera **/
    double fovy_;

    /** The distance to the near-clip-plane of the used camera **/
    double ncp_;

     /** The distance to the far-clip-plane of the used camera **/
    double fcp_;

    /** The orientaton of the visual axis of the camera **/
    const Eigen::Vector3d camera_orientation_;

    /**
     * \brief Returns a rating of an object's pose based on the distance to the camera
     *
     * \param object_pose       The pose of the object
     * \return                  The calculated rating
     */
    double getDistanceRating(const geometry_msgs::Pose &object_pose);

    /**
     * \brief Returns a rating of an object's pose based on the (azimut or elevation) angle between the camera's visual axis and the vector from the camera to the object's position.
     *
     * \param object_pose       The pose of the object
     * \param direction         True == azimut, false == elevation.
     * \return                  The calculated rating
     */
    double getAngleRating(const geometry_msgs::Pose &object_pose, bool direction);

public:
    /**
     * \brief The constructor of the class
     *
     * \param fovx      The field of view in x-direction
     * \param fovy      The field of view in y-direction
     * \param ncp       The distance to the near-clip-plane
     * \param fcp       The distance to the far-clip-plane
     */
    Rating(double fovx, double fovy, double ncp, double fcp);

    /**
     * \brief Return whether a pose is visible in the camera frustum based on the distance and angle rating
     *
     * \param pose          The given object's pose
     * \param threshold_d   The minimum distance rating value this pose needs to have to be visible.
     * \param threshold_x   The minimum angle rating value in azimut this pose needs to have to be visible.
     * \param threshold_y   The minimum angle rating value in elevation this pose needs to have to be visible.
     * \return              True if the object is visible based on the rating, false otherwise
     */
    bool ratePose(const geometry_msgs::Pose &pose, double threshold_d, double threshold_x, double threshold_y);
};

}

#endif /* RATING_H */

