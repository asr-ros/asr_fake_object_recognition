/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FAKE_OBJECT_RECOGNITION_H
#define FAKE_OBJECT_RECOGNITION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <object_config.h>
#include <asr_fake_object_recognition/GetRecognizer.h>
#include <asr_fake_object_recognition/ReleaseRecognizer.h>
#include <asr_fake_object_recognition/GetAllRecognizers.h>
#include <asr_fake_object_recognition/ClearAllRecognizers.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <asr_msgs/AsrObject.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <asr_fake_object_recognition/FakeObjectRecognitionConfig.h>
#include <error_simulation.h>
#include <std_msgs/ColorRGBA.h>

namespace fake_object_recognition {

using namespace asr_fake_object_recognition;

/** The name of this package **/
const static std::string NODE_NAME("asr_fake_object_recognition");

/** The name of service used for adding objects to the list of recognizable objects **/
const static std::string GET_RECOGNIZER_SERVICE_NAME("get_recognizer");

/** The name of service used for removing objects from the list of recognizable objects **/
const static std::string RELEASE_RECOGNIZER_SERVICE_NAME("release_recognizer");

/** The name of service used for adding all objects from config.xml to recognizable objects **/
const static std::string GET_ALL_RECOGNIZERS_SERVICE_NAME("get_all_recognizers");

/** The name of service used for removing objects from the list of recognizable objects **/
const static std::string CLEAR_ALL_RECOGNIZERS_SERVICE_NAME("clear_all_recognizers");

/**
*   \brief The central class of the recognition system used for managing the ros subscriptions,
*           configuration changes, loading of the objects, the recognition itself and the visualisation of the results
*/
class FakeObjectRecognition {

private:
    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh_;

    /** Dynamic reconfigure server which keeps track of the callback function */
    dynamic_reconfigure::Server<FakeObjectRecognitionConfig> reconfigure_server_;

    /** The configuration containing the dynamic parameters of this process **/
    FakeObjectRecognitionConfig config_;

    /** This value indicates whether the configuration has changed **/
    bool config_changed_;

    /** Ros service handlers used for handling requests */
    ros::ServiceServer get_recognizer_service_;
    ros::ServiceServer release_recognizer_service_;

    ros::ServiceServer get_all_recognizers_service_;
    ros::ServiceServer clear_all_recognizers_service_;


    /** Ros publishers which manage the advertisement of specific topics */
    ros::Publisher recognized_objects_pub_;
    ros::Publisher recognized_objects_marker_pub_;
    ros::Publisher generated_constellation_marker_pub_;

    /** A timer which is used to control the recognition cycles **/
    ros::Timer timer_;

    /** This value indicates whether the recognition is paused **/
    bool recognition_released_;

    /** The parameters of the used cameras **/
    double fovx_;
    double fovy_;
    double ncp_;
    double fcp_;

    /** The used frames (world and cameras) **/
    std::string frame_world_;
    std::string frame_camera_left_;
    std::string frame_camera_right_;

    /** The path to the file containing information about the available objects **/
    std::string config_file_path_;

    /** The topics of the publishers **/
    std::string output_rec_objects_topic_;
    std::string output_rec_markers_topic_;
    std::string output_constellation_marker_topic_;

    /** The minimum value a distance rating of a pose needs to have to be valid **/
    double rating_threshold_d_;
    /** The minimum value an azimut angle rating of a pose needs to have to be valid **/
    double rating_threshold_x_;
    /** The minimum value an elevation angle rating of a pose needs to have to be valid **/
    double rating_threshold_y_;

    /** The time between recognition cycles used by the timer **/
    double timer_duration_;

    /** The available objects **/
    std::vector<ObjectConfig> objects_;

    /** The objects which shall be recognized **/
    std::vector<std::string> objects_to_rec_;

    /** A listener used to transform between the world and camera frames **/
    tf::TransformListener listener_;

    /** An error simulator used to generate pose errors **/
    ErrorSimulation err_sim_;



    /**
    * \brief  This function loads the objects of the object-config-file
    */
    void loadObjects();

    /**
    * \brief  This function parses the string containing the pose of an entry in the object-config-file
    *
    * \param pose_in    The given pose string
    * \param pose_out   The parsed pose
    * \param delim      The string used to split the input pose-string
    */
    bool parsePoseString(std::string pose_in, geometry_msgs::Pose &pose_out, std::string delim, std::string angles);

    /**
     * Processes the request to recognize the given object
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res);

    /**
     * Processes the request to release a previously created recognizer
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res);

    /**
     * Processes the request to recognize all objects loaded from config.xml
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processGetAllRecognizersRequest(GetAllRecognizers::Request &req, GetAllRecognizers::Response &res);

    /**
     * Processes the request to release all recognizer
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processClearAllRecognizers(ClearAllRecognizers::Request &req, ClearAllRecognizers::Response &res);


    /**
     * \brief The callback function of the timer
     *
     * \param event     Structure passed as a parameter to the callback invoked by a ros::Timer
     */
    void timerCallback(const ros::TimerEvent &event);

    /**
     * \brief The callback function which is called when the configuration has changed
     *
     * \param config    The new configuration
     * \param level     The level which is the result of ORing together all of level values of the parameters that have changed
     */
    void configCallback(FakeObjectRecognitionConfig &config, uint32_t level);

    /**
     * \brief This function is called whenever objects shall be recognized
     */
    void doRecognition();

    /**
     * \brief Checks whether an object is currently visible
     *
     * \param pose_left     The pose of the object in the left camera frame
     * \param pose_right    The pose of the object in the right camera frame
     * \return              True if the object is visible, false otherwise
     */
    bool objectIsVisible(const geometry_msgs::Pose &pose_left, const geometry_msgs::Pose &pose_right);

    /**
     * \brief Transforms a given pose from one frame to another
     *
     * \param pose          The given pose
     * \param frame_from    The frame the given pose belongs to
     * \param frame_to      The frame the pose is transformed to
     * \return              The transformed pose
     */
    geometry_msgs::Pose transformFrame(const geometry_msgs::Pose &pose, const std::string &frame_from, const std::string &frame_to);

    /**
     * \brief Creates a AsrObject-message for a found object
     *
     * \param object_config     The configuration file of the found object
     * \param pose              The pose of the found object
     * \param frame_id          The frame the given pose belongs to
     * \return                  The created AsrObject-message
     */
    asr_msgs::AsrObjectPtr createAsrMessage(const ObjectConfig &object_config, const geometry_msgs::Pose &pose, const std::string &frame_id);

    /**
     * \brief Creates a visualization marker for a found object
     *
     * \param object            The AsrObject-message created for the found object
     * \param id                The id used to distinguish between multiple markers
     * \param lifetime          The lifetime of the marker
     * \param use_col_init      If this is true the marker has a special color indicating that this is a marker visualizing an available object
     * \return                  The created marker
     */
    visualization_msgs::Marker createMarker(const asr_msgs::AsrObjectPtr &object, int id, int lifetime, bool use_col_init = false);

    /**
     * \brief Returns the color of a mesh based on the object's id (Only creates a color if it is an object of the segmentable category
     *
     * \param observed_id       The id of an object
     * \return                  The correlating color
     */
    static std_msgs::ColorRGBA getMeshColor(std::string observed_id);

    /**
     * \brief Creates a std_msgs::ColorRGBA-message from the given values
     *
     * \param red       The R-value of the RGBA-color
     * \param green     The G-value of the RGBA-color
     * \param blue      The B-value of the RGBA-color
     * \param alpha     The A-value of the RGBA-color
     * \return          The created color-message
     */
    static std_msgs::ColorRGBA createColorRGBA(float red, float green, float blue, float alpha);

public:
    /**
     * \brief The constructor of this class
     */
    FakeObjectRecognition();

};

}


#endif /* FAKE_OBJECT_RECOGNITION_H */

