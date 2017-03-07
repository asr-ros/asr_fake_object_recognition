/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ERROR_SIMULATION_H
#define ERROR_SIMULATION_H

#include <geometry_msgs/Pose.h>

namespace fake_object_recognition {

/** The default value of the pose invalidation probability **/
const static double DEFAULT_PROB_POSE_INVAL(0.01);

/** The default value of the normal distribution's mean value used for position errors **/
const static double DEFAULT_POS_NOISE_MEAN(0.0);

/** The default value of the normal distribution's standard deviation value used for position errors **/
const static double DEFAULT_POS_NOISE_DEV(0.005);

/** The default value of the normal distribution's mean value used for orientation errors (x-axis) **/
const static double DEFAULT_OR_X_NOISE_MEAN(0.0);

/** The default value of the normal distribution's standard deviation value used for orientation errors (x-axis) **/
const static double DEFAULT_OR_X_NOISE_DEV(0.02);

/** The default value of the normal distribution's mean value used for orientation errors (y-axis) **/
const static double DEFAULT_OR_Y_NOISE_MEAN(0.0);

/** The default value of the normal distribution's standard deviation value used for orientation errors (y-axis) **/
const static double DEFAULT_OR_Y_NOISE_DEV(0.02);

/** The default value of the normal distribution's mean value used for orientation errors (z-axis) **/
const static double DEFAULT_OR_Z_NOISE_MEAN(0.0);

/** The default value of the normal distribution's standard deviation value used for orientation errors (y-axis) **/
const static double DEFAULT_OR_Z_NOISE_DEV(0.02);

/**
 * \brief This class is used to simulate typical errors of an object recognition system
 */
class ErrorSimulation {

    /** If this is true pose invalidation errors are calculated **/
    bool use_pose_invalidation_;

    /** If this is true position errors are calculated **/
    bool use_position_noise_;

    /** If this is true orientation errors are calculated **/
    bool use_orientation_noise_;

    /** The value of the pose invalidation probability **/
    double prob_pose_inval_;

    /** The value of the normal distribution's mean value used for position errors **/
    double pos_noise_dist_mean_;

    /** The value of the normal distribution's standard deviation value used for position errors **/
    double pos_noise_dist_dev_;


    /** The value of the normal distribution's mean value used for orientation errors (x-axis) **/
    double or_x_noise_dist_mean_;

    /** The value of the normal distribution's standard deviation value used for orientation errors (x-axis) **/
    double or_x_noise_dist_dev_;

    /** The value of the normal distribution's mean value used for orientation errors (y-axis) **/
    double or_y_noise_dist_mean_;

    /** The value of the normal distribution's standard deviation value used for orientation errors (y-axis) **/
    double or_y_noise_dist_dev_;

    /** The value of the normal distribution's mean value used for orientation errors (z-axis) **/
    double or_z_noise_dist_mean_;

    /** The value of the normal distribution's standard deviation value used for orientation errors (z-axis) **/
    double or_z_noise_dist_dev_;


    /**
     * \brief Returns a value calculated on a uniform distribution
     * \param interval_start    The lower value of the interval used for the uniform distribution
     * \param interval_end      The upper value of the interval used for the uniform distribution
     * \return                  The calculated value
     */
    double genUniformNumber(double interval_start, double interval_end);

    /**
     * \brief Returns a value calculated on a normal distribution
     * \param interval_start    The mean value of the normal distribution
     * \param interval_end      The standard deviation value of the normal distribution
     * \return                  The calculated value
     */
    double genNormalDistNumber(double mean, double std_deviation);


public:
    /**
     * \brief The constructor of this class
     */
    ErrorSimulation();

    /**
     * \brief The constructor of this class (with parameters)
     * \param use_pose_invalidation     Indicates if pose invalidation is used
     * \param use_position_noise        Indicates if position errors are used
     * \param use_orientation_noise     Indicates if orientation errors are used
     */
    ErrorSimulation(bool use_pose_invalidation, bool use_position_noise, bool use_orientation_noise);

    /**
     * @brief Invalidates a pose by the probability member of this class
     * @return  True if the pose is calculated as invalid, false otherwise
     */
    bool poseInvalidation();

    /**
     * \brief Adds error values to the given pose (position only)
     * \param pose      The given pose
     * \return          The pose with errors
     */
    geometry_msgs::Pose addNoiseToPosition(const geometry_msgs::Pose &pose);

    /**
     * \brief Adds error values to the given pose (orientation only)
     * \param pose      The given pose
     * \return          The pose with errors
     */
    geometry_msgs::Pose addNoiseToOrientation(const geometry_msgs::Pose &pose);

    /**
     * \brief Set the probability of the pose invalidation
     *
     * \param prob_pose_inval       The probability to set
     */
    void setProbPoseInval(double prob_pose_inval);

    /**
     * \brief Sets the mean value of the normal distribution used by the position error generation
     *
     * \param pos_noise_dist_mean       The mean value to set
     */
    void setPoseNoiseDistMean(double pos_noise_dist_mean);

    /**
     * \brief Sets the standard deviation value of the normal distribution used by the position error generation
     *
     * \param pos_noise_dist_mean       The standard deviation value to set
     */
    void setPoseNoiseDistDev(double pos_noise_dist_dev);

    /**
     * \brief Sets the mean value of the normal distribution used by the orientation error generation (x-axis)
     *
     * \param or_x_noise_dist_mean       The mean value to set
     */
    void setOrXNoiseDistMean(double or_x_noise_dist_mean);

    /**
     * \brief Sets the standard deviation value of the normal distribution used by the orientation error generation (x-axis)
     * \param or_x_noise_dist_dev       The standard deviation value to set
     */
    void setOrXNoiseDistDev(double or_x_noise_dist_dev);

    /**
     * \brief Sets the mean value of the normal distribution used by the orientation error generation (y-axis)
     *
     * \param or_y_noise_dist_mean       The mean value to set
     */
    void setOrYNoiseDistMean(double or_y_noise_dist_mean);

    /**
     * \brief Sets the standard deviation value of the normal distribution used by the orientation error generation (y-axis)
     * \param or_y_noise_dist_dev       The standard deviation value to set
     */
    void setOrYNoiseDistDev(double or_y_noise_dist_dev);

    /**
     * \brief Sets the mean value of the normal distribution used by the orientation error generation (z-axis)
     *
     * \param or_z_noise_dist_mean       The mean value to set
     */
    void setOrZNoiseDistMean(double or_z_noise_dist_mean);

    /**
     * \brief Sets the standard deviation value of the normal distribution used by the orientation error generation (z-axis)
     * \param or_z_noise_dist_dev       The standard deviation value to set
     */
    void setOrZNoiseDistDev(double or_z_noise_dist_dev);
};

}

#endif /* ERROR_SIMULATION_H */
