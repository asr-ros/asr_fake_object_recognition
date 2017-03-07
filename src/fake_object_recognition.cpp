/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "fake_object_recognition.h"
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <ros/package.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "rating.h"
#include "error_simulation.h"
#include <unistd.h>
#include <algorithm>
//Eigen
#include <Eigen/Geometry>

#include "object_database/ObjectMetaData.h"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

namespace fake_object_recognition {

FakeObjectRecognition::FakeObjectRecognition() : nh_(NODE_NAME), config_changed_(false), recognition_released_(true) {
    ROS_DEBUG("Initialize process");
    nh_.getParam("fovx", fovx_); // Field of view in x.
    nh_.getParam("fovy", fovy_); // Field of view in y.
    nh_.getParam("ncp", ncp_); // Near clipping plane of fustrum.
    nh_.getParam("fcp", fcp_); // Far clipping plane of fustrum.
    nh_.getParam("frame_world", frame_world_);
    nh_.getParam("frame_camera_left", frame_camera_left_);
    nh_.getParam("frame_camera_right", frame_camera_right_);
    nh_.getParam("config_file_path", config_file_path_);
    nh_.getParam("output_rec_objects_topic", output_rec_objects_topic_);
    nh_.getParam("output_rec_marker_topic", output_rec_markers_topic_);
    nh_.getParam("output_constellation_topic", output_constellation_marker_topic_);
    nh_.getParam("rating_threshold_d", rating_threshold_d_);
    nh_.getParam("rating_threshold_x", rating_threshold_x_);
    nh_.getParam("rating_threshold_y", rating_threshold_y_);
    nh_.getParam("timer_duration", timer_duration_);

    reconfigure_server_.setCallback(boost::bind(&FakeObjectRecognition::configCallback, this, _1, _2));

    get_recognizer_service_ = nh_.advertiseService(GET_RECOGNIZER_SERVICE_NAME, &FakeObjectRecognition::processGetRecognizerRequest, this);
    release_recognizer_service_ = nh_.advertiseService(RELEASE_RECOGNIZER_SERVICE_NAME, &FakeObjectRecognition::processReleaseRecognizerRequest, this);

    get_all_recognizers_service_ = nh_.advertiseService(GET_ALL_RECOGNIZERS_SERVICE_NAME, &FakeObjectRecognition::processGetAllRecognizersRequest, this);
    clear_all_recognizers_service_ = nh_.advertiseService(CLEAR_ALL_RECOGNIZERS_SERVICE_NAME, &FakeObjectRecognition::processClearAllRecognizers, this);

    recognized_objects_pub_ = nh_.advertise<asr_msgs::AsrObject>(output_rec_objects_topic_, 1);
    recognized_objects_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_rec_markers_topic_, 1);
    generated_constellation_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_constellation_marker_topic_, 1);
    object_normals_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/nbv/object_normals", 100);

    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &FakeObjectRecognition::timerCallback, this);

    //Only used here to visualize object constellation. The rest (errors etc) is done at first doRecognition() call.
    loadObjects();

    // initialize bounding boxes and normals:
    ROS_DEBUG_STREAM("Initializing object bounding boxes and normals");
    // Set up service to get normals:
    ros::service::waitForService("/object_database/object_meta_data");
    object_metadata_service_client_ = nh_.serviceClient<object_database::ObjectMetaData>("/object_database/object_meta_data");
    // Set bounding box corner point filename:
    bb_corners_file_name_ = ros::package::getPath("asr_fake_object_recognition") + "/config/bounding_box_corners.xml";

    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
       if (normals_.find(iter->getType()) == normals_.end()) {    // if normals of object type not yet initialized
           ROS_DEBUG_STREAM("Initializing normals of object type " << iter->getType());
           normals_[iter->getType()] = getNormals(*iter);
           ROS_DEBUG_STREAM("Found " << normals_[iter->getType()].size() << " normals. Normals initialized.");
       }
       if (bounding_box_corners_.find(iter->getType()) == bounding_box_corners_.end()) { // if bounding box corners of object type not yet initialized
           ROS_DEBUG_STREAM("Initializing bounding box of object type " << iter->getType());
           std::array<geometry_msgs::Point, 8> corner_points;
           if (!getBBfromFile(corner_points, iter->getType())) { // No corners were found in file: calculate and write them to file
               corner_points = calculateBB(*iter);
           }
           bounding_box_corners_[iter->getType()] = corner_points;
           ROS_DEBUG_STREAM("Found " << bounding_box_corners_[iter->getType()].size() << " bounding box corner points. Bounding box initialized.");
        }
    }
    ROS_INFO("Recognition is initially released");
}

void FakeObjectRecognition::loadObjects() {
    objects_.clear();

    std::string xml_path;
    if (boost::starts_with(config_file_path_, ".")) //if path starts with a point, assume it's a relative path.
    {
        xml_path = ros::package::getPath("asr_fake_object_recognition") + config_file_path_.substr(1);
    }
    else //Otherwise use it as an absolute path
    {
        xml_path = config_file_path_;
    }
    ROS_DEBUG_STREAM("Path to objects.xml: " << xml_path);

    try {
        rapidxml::file<> xmlFile(xml_path.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        rapidxml::xml_node<> *root_node = doc.first_node();
        if (root_node) {
            rapidxml::xml_node<> *child_node = root_node->first_node();
            while (child_node) {
                rapidxml::xml_attribute<> *type_attribute = child_node->first_attribute("type");
                rapidxml::xml_attribute<> *id_attribute = child_node->first_attribute("id");
                rapidxml::xml_attribute<> *mesh_attribute = child_node->first_attribute("mesh");
                rapidxml::xml_attribute<> *angles_attribute = child_node->first_attribute("angles");
                if (type_attribute && id_attribute && mesh_attribute && angles_attribute) {
                    std::string type = type_attribute->value();
                    std::string mesh = mesh_attribute->value();
                    std::string id = id_attribute->value();
                    std::string pose_string = child_node->value();
                    std::string angles = angles_attribute->value();
                    geometry_msgs::Pose pose;
                    if (parsePoseString(pose_string, pose, " ,", angles)) {
                        objects_.push_back(ObjectConfig(type, id, pose, mesh));
                    }
                }
                child_node = child_node->next_sibling();
            }
        }
    } catch(std::runtime_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
    } catch (rapidxml::parse_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
    }

}


bool FakeObjectRecognition::parsePoseString(std::string pose_in, geometry_msgs::Pose &pose_out, std::string delim, std::string angles) {
    std::vector<std::string> strvec;

    boost::algorithm::trim_if(pose_in, boost::algorithm::is_any_of(delim));
    boost::algorithm::split(strvec, pose_in, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
    if (strvec.size() == 6 || strvec.size() == 7) {
        try {
            pose_out.position.x = boost::lexical_cast<double>(strvec[0]);
            pose_out.position.y = boost::lexical_cast<double>(strvec[1]);
            pose_out.position.z = boost::lexical_cast<double>(strvec[2]);

            if(angles == "quaternion" && strvec.size() == 7)
            {
                pose_out.orientation.w = boost::lexical_cast<double>(strvec[3]);
                pose_out.orientation.x = boost::lexical_cast<double>(strvec[4]);
                pose_out.orientation.y = boost::lexical_cast<double>(strvec[5]);
                pose_out.orientation.z = boost::lexical_cast<double>(strvec[6]);
            }
            else if(angles == "euler" && strvec.size() == 6)
            {
                double euler0,euler1,euler2;
                euler0 = boost::lexical_cast<double>(strvec[3]);
                euler1 = boost::lexical_cast<double>(strvec[4]);
                euler2 = boost::lexical_cast<double>(strvec[5]);

                Eigen::Matrix3d rotationMatrix;
                rotationMatrix = Eigen::AngleAxisd(euler0 * (M_PI / 180), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(euler1 * (M_PI / 180), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler2 * (M_PI / 180), Eigen::Vector3d::UnitZ());

                Eigen::Quaternion<double> result(rotationMatrix);
                pose_out.orientation.w = result.w();
                pose_out.orientation.x = result.x();
                pose_out.orientation.y = result.y();
                pose_out.orientation.z = result.z();
            }
            else
            {
                ROS_ERROR("Invalid XML syntax.");
                nh_.shutdown();
            }

            return true;
        } catch (boost::bad_lexical_cast err) {
            ROS_DEBUG_STREAM("Can't cast node-value. Cast error: " << err.what());
        }
    }
    return false;
}

bool FakeObjectRecognition::processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res) {
    ROS_DEBUG("Get recognizer request");
    if(std::find(objects_to_rec_.begin(), objects_to_rec_.end(), req.object_type_name) != objects_to_rec_.end()) {
        ROS_DEBUG("Requested object is already in list of recognizable objects");
    } else {
        ROS_DEBUG("Add requested object to recognizable object list");
        objects_to_rec_.push_back(req.object_type_name);
        recognition_released_ = false;
    }
    return true;
}

bool FakeObjectRecognition::processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res) {
    ROS_DEBUG("Release recognizer request");
    objects_to_rec_.erase(std::remove(objects_to_rec_.begin(), objects_to_rec_.end(), req.object_type_name), objects_to_rec_.end());
    if (!(objects_to_rec_.size() > 0)) {
        recognition_released_ = true;
    }
    return true;
}

bool FakeObjectRecognition::processGetAllRecognizersRequest(GetAllRecognizers::Request &req, GetAllRecognizers::Response &res) {
    ROS_DEBUG("Get all recognizers request");

    objects_to_rec_.clear();
    for(std::vector<ObjectConfig>::iterator objectIt = objects_.begin(); objectIt != objects_.end(); objectIt++){
        ROS_DEBUG_STREAM("Adding objects " << objectIt->getType() << " to recognizable list");
        objects_to_rec_.push_back(objectIt->getType());

    }

    recognition_released_ = false;
    return true;

}

bool FakeObjectRecognition::processClearAllRecognizers(ClearAllRecognizers::Request &req, ClearAllRecognizers::Response &res) {
    ROS_DEBUG("Clear all recognizers request");
    objects_to_rec_.clear();
    recognition_released_ = true;
    return true;
}

void FakeObjectRecognition::timerCallback(const ros::TimerEvent& event) {
    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
        //Generating marker for each object that is present in config file.
        asr_msgs::AsrObjectPtr asr_msg = createAsrMessage(*iter, iter->getPose(), frame_world_);
        generated_constellation_marker_pub_.publish(createMarker(asr_msg, iter - objects_.begin(), 2 * timer_duration_, true));
		// Generating normal markers for each object.
        visualization_msgs::MarkerArray::Ptr normal_markers = createNormalMarker(*iter, (iter - objects_.begin()) * 100, 10 * timer_duration_);
        object_normals_pub_.publish(normal_markers); // can have up to 100 normals
    }
    if (!recognition_released_) {
        doRecognition();
    }
}

void FakeObjectRecognition::configCallback(FakeObjectRecognitionConfig &config, uint32_t level) {
    config_changed_ = true;
    config_ = config;
}

void FakeObjectRecognition::doRecognition() {
    ROS_INFO("Do recognition");
    // React to changed config:
    if (config_changed_) {
        ROS_DEBUG("Configuration change");
        ROS_DEBUG("Load Objects");
        if (config_file_path_ != config_.config_file_path) {
            ROS_INFO_STREAM("Path to config file has changed. Recognition is released");
            config_file_path_ = config_.config_file_path;
            objects_to_rec_.clear();
            recognition_released_ = true;
            return;
        }
        loadObjects();
        err_sim_ = ErrorSimulation(config_.use_pose_invalidation, config_.use_position_noise, config_.use_orientation_noise);
        err_sim_.setProbPoseInval(config_.prob_pose_invalidation);
        err_sim_.setPoseNoiseDistMean(config_.pos_noise_normal_dist_mean);
        err_sim_.setPoseNoiseDistDev(config_.pos_noise_normal_dist_dev);
        err_sim_.setOrXNoiseDistMean(config_.or_x_noise_normal_dist_mean);
        err_sim_.setOrXNoiseDistDev(config_.or_x_noise_normal_dist_dev);
        err_sim_.setOrYNoiseDistMean(config_.or_y_noise_normal_dist_mean);
        err_sim_.setOrYNoiseDistDev(config_.or_y_noise_normal_dist_dev);
        err_sim_.setOrZNoiseDistMean(config_.or_z_noise_normal_dist_mean);
        err_sim_.setOrZNoiseDistDev(config_.or_z_noise_normal_dist_dev);
        config_changed_ = false;
    }
    // Print all loaded objects to debug stream:
    std::string recognizable_objects;
    std::string objects_to_rec;
    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
        recognizable_objects += iter->getType() + " ";
    }
    ROS_DEBUG_STREAM("All recognizable objects: " << recognizable_objects);
    // Print all Objects for which recognition has been requested (via processGetRecognizerRequest or processGetAllRecognizersRequest) to debug stream:
    for (std::vector<std::string>::iterator iter = objects_to_rec_.begin(); iter != objects_to_rec_.end(); ++iter) {
        objects_to_rec += *iter + " ";
    }
    ROS_DEBUG_STREAM("Objects to recognize: " << objects_to_rec);

    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {             // for all loaded (recognizable) objects:
        if(std::find(objects_to_rec_.begin(), objects_to_rec_.end(), iter->getType()) == objects_to_rec_.end()) {   // unless end of vector of requested objects is reached:
            continue;
        }
        ROS_INFO_STREAM("Current object: '" << iter->getType() << "'");
        geometry_msgs::Pose pose_left;
        geometry_msgs::Pose pose_right;

        std::array<geometry_msgs::Point, 8> bounding_box = bounding_box_corners_[iter->getType()]; // Bounding box, given as corner points relative to object frame
        std::array<geometry_msgs::Point, 8> bb_left;
        std::array<geometry_msgs::Point, 8> bb_right;

        std::vector<geometry_msgs::Point> normals = normals_[iter->getType()]; // Object normals
        std::vector<geometry_msgs::Point> normals_left;
        std::vector<geometry_msgs::Point> normals_right;

        try {
            if (config_.use_camera_pose) { // If camera pose is used:
                ROS_DEBUG_STREAM("Transform into camera frame");

                pose_left = transformFrame((*iter).getPose(), frame_world_, frame_camera_left_);
                pose_right = transformFrame((*iter).getPose(), frame_world_, frame_camera_right_);

                ROS_DEBUG_STREAM("Pose left: " << pose_left.position.x << " " << pose_left.position.y << " " << pose_left.position.z << " " << pose_left.orientation.w << " " << pose_left.orientation.x << " " << pose_left.orientation.y << " " << pose_left.orientation.z << " ");
                ROS_DEBUG_STREAM("Pose right: " << pose_right.position.x << " " << pose_right.position.y << " " << pose_right.position.z << " " << pose_right.orientation.w << " " << pose_right.orientation.x << " " << pose_right.orientation.y << " " << pose_right.orientation.z << " ");

                pose_left = err_sim_.addNoiseToOrientation(err_sim_.addNoiseToPosition(pose_left));
                pose_right = err_sim_.addNoiseToOrientation(err_sim_.addNoiseToPosition(pose_right));

                ROS_DEBUG_STREAM("Pose left (with errors): " << pose_left.position.x << " " << pose_left.position.y << " " << pose_left.position.z << " " << pose_left.orientation.w << " " << pose_left.orientation.x << " " << pose_left.orientation.y << " " << pose_left.orientation.z << " ");
                ROS_DEBUG_STREAM("Pose right (with errors): " << pose_right.position.x << " " << pose_right.position.y << " " << pose_right.position.z << " " << pose_right.orientation.w << " " << pose_right.orientation.x << " " << pose_right.orientation.y << " " << pose_right.orientation.z << " ");

                // for bounding box and normals transformation:
                Eigen::Quaterniond rot_left(pose_left.orientation.w, pose_left.orientation.x, pose_left.orientation.y, pose_left.orientation.z);
                Eigen::Quaterniond rot_right(pose_right.orientation.w, pose_right.orientation.x, pose_right.orientation.y, pose_right.orientation.z);
                // Transform bounding box corner points:
                ROS_DEBUG_STREAM("Transform " << bounding_box.size() << " bounding box corner points.");
                Eigen::Vector3d trans_left(pose_left.position.x, pose_left.position.y, pose_left.position.z);
                Eigen::Vector3d trans_right(pose_right.position.x, pose_right.position.y, pose_right.position.z);
                bb_left = transformPoints(bounding_box, rot_left, trans_left);
                bb_right = transformPoints(bounding_box, rot_right, trans_right);

                //Transform normals (Rotate according to object pose):
                ROS_DEBUG_STREAM("Transform " << normals.size() << " normals.");
                normals_left = transformPoints(normals, rot_left, Eigen::Vector3d(0.0, 0.0, 0.0));
                normals_right = transformPoints(normals, rot_right, Eigen::Vector3d(0.0, 0.0, 0.0));
            } else {
                ROS_DEBUG_STREAM("Camera pose is not used");
            }
        } catch (tf2::LookupException &exc) {
            ROS_DEBUG_STREAM("Lookup exception at doRecognition: " << exc.what());
            continue;
        } catch (tf2::ExtrapolationException &exc) {
            ROS_DEBUG_STREAM("Extrapolation exception at doRecognition: " << exc.what());
            continue;
        } catch (tf2::InvalidArgumentException &exc) {
            ROS_DEBUG_STREAM("Invalid argument exception at doRecognition: " << exc.what());
            continue;
        } catch (tf2::ConnectivityException &exc) {
            ROS_DEBUG_STREAM("Connectivity exception at doRecognition: " << exc.what());
            continue;
        }

        //if (!(config_.use_camera_pose) || objectIsVisible(pose_left, pose_right)) { // old way of doing the following
        if (!(config_.use_camera_pose) || objectIsVisible(bb_left, bb_right, pose_left, pose_right, normals_left, normals_right)) { // If camera pose is not used OR camera pose is used and objectIsVisible() returns true:
            ROS_INFO_STREAM("Object '" << (*iter).getType() << "' was found");
            asr_msgs::AsrObjectPtr asr_msg;
            if (!(config_.use_camera_pose)) {
                asr_msg = createAsrMessage(*iter, iter->getPose(), frame_world_);
            } else {
                asr_msg = createAsrMessage(*iter, pose_left, frame_camera_left_);
            }
            recognized_objects_pub_.publish(asr_msg);
            recognized_objects_marker_pub_.publish(createMarker(asr_msg, iter - objects_.begin(), 10 * timer_duration_));
        }
    }
    ROS_INFO("---------------------------------- \n");
}

geometry_msgs::Pose FakeObjectRecognition::transformFrame(const geometry_msgs::Pose &pose, const std::string &frame_from, const std::string &frame_to) {
    //lookup transform to get latest timestamp
    tf::StampedTransform transform;
    listener_.lookupTransform(frame_from, frame_to, ros::Time(0), transform);

    geometry_msgs::PoseStamped stamped_in;
    stamped_in.header.frame_id = frame_from;
    stamped_in.header.stamp = transform.stamp_;
    stamped_in.pose.position = pose.position;
    stamped_in.pose.orientation = pose.orientation;

    geometry_msgs::PoseStamped stamped_out;
    listener_.transformPose(frame_to, stamped_in, stamped_out);

    geometry_msgs::Pose result;
    result.position = stamped_out.pose.position;
    result.orientation = stamped_out.pose.orientation;

    return result;
}
// Currently not used anymore:
bool FakeObjectRecognition::objectIsVisible(const geometry_msgs::Pose &pose_left, const geometry_msgs::Pose &pose_right) {
    Rating rating(fovx_, fovy_, ncp_, fcp_);
    bool pose_inval = err_sim_.poseInvalidation();
    if (!pose_inval) {ROS_DEBUG("Pose is invalid (simulated error)");}

    bool rating_result = false;
    ROS_DEBUG("Rating in left camera:");
    bool left_rating = rating.ratePose(pose_left, rating_threshold_d_, rating_threshold_x_, rating_threshold_y_);
    ROS_DEBUG("Rating in left camera:");
    bool right_rating = rating.ratePose(pose_right,  rating_threshold_d_, rating_threshold_x_, rating_threshold_y_);

    switch(config_.frustum_mode) {
    case 1: rating_result = left_rating | right_rating; ROS_DEBUG("Rating left and right pose (OR)"); break;
    case 2: rating_result = left_rating; ROS_DEBUG("Rating left pose only"); break;
    case 3: rating_result = right_rating; ROS_DEBUG("Rating right pose only"); break;
    default: rating_result = left_rating & right_rating; ROS_DEBUG("Rating left and right pose (AND)");
    }

    
    if (rating_result && pose_inval) {
        return true;
    }
    return false;
}
// Currently used:
bool FakeObjectRecognition::objectIsVisible(const std::array<geometry_msgs::Point, 8> &bb_left, const std::array<geometry_msgs::Point, 8> &bb_right,
                                            const geometry_msgs::Pose &pose_left, const geometry_msgs::Pose &pose_right,
                                            const std::vector<geometry_msgs::Point> &normals_left, const std::vector<geometry_msgs::Point> &normals_right) {
    Rating rating(fovx_, fovy_, ncp_, fcp_);
    bool pose_inval = err_sim_.poseInvalidation();
    if (!pose_inval) {ROS_DEBUG("Pose is invalid (simulated error)");}

    bool rating_result = false;
    bool left_rating = rating.rateBBandNormal(pose_left, bb_left, normals_left, rating_threshold_);
    bool right_rating = rating.rateBBandNormal(pose_right, bb_right, normals_right, rating_threshold_);


    switch(config_.frustum_mode) {
    case 1: rating_result = left_rating | right_rating; ROS_DEBUG("Rating left and right pose (OR)"); break;
    case 2: rating_result = left_rating; ROS_DEBUG("Rating left pose only"); break;
    case 3: rating_result = right_rating; ROS_DEBUG("Rating right pose only"); break;
    default: rating_result = left_rating & right_rating; ROS_DEBUG("Rating left and right pose (AND)");
    }


    if (rating_result && pose_inval) {
        return true;
    }
    return false;
}

asr_msgs::AsrObjectPtr FakeObjectRecognition::createAsrMessage(const ObjectConfig &object_config, const geometry_msgs::Pose &pose, const std::string &frame_id) {
    asr_msgs::AsrObjectPtr object;
    object.reset(new asr_msgs::AsrObject());

    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = ros::Time::now();
    object->header = header;
    object->providedBy = "fake_object_recognition";

    geometry_msgs::PoseWithCovariance pose_covariance;
    pose_covariance.pose.position.x = pose.position.x;
    pose_covariance.pose.position.y = pose.position.y;
    pose_covariance.pose.position.z = pose.position.z;
    pose_covariance.pose.orientation.w = pose.orientation.w;
    pose_covariance.pose.orientation.x = pose.orientation.x;
    pose_covariance.pose.orientation.y = pose.orientation.y;
    pose_covariance.pose.orientation.z = pose.orientation.z;

    for(unsigned int i = 0; i < pose_covariance.covariance.size(); i++) {
        pose_covariance.covariance.at(i) = 0.0f;
    }
    object->sampledPoses.push_back(pose_covariance);

    // Bounding Box:
    boost::array< ::geometry_msgs::Point_<std::allocator<void> > , 8> bounding_box;
    std::array<geometry_msgs::Point, 8> object_bb = bounding_box_corners_[object_config.getType()];
    for (unsigned int i = 0; i < 8; i++) {
        bounding_box[i] = object_bb.at(i);
    }
    object->boundingBox = bounding_box;

    if(object_config.getId() == std::string("0"))
    {
        object->colorName = "textured";
    }
    else
    {
        object->color = getMeshColor(object_config.getId());
    }
    object->type = object_config.getType();

    object->identifier = object_config.getId();
    object->meshResourcePath = object_config.getMeshName();

    //object->sizeConfidence = pose_rec->getResults().at(results_index)->getScore3D();
    //object->typeConfidence = pose_rec->getResults().at(results_index)->getScore2D();

    return object;
}

visualization_msgs::Marker FakeObjectRecognition::createMarker(const asr_msgs::AsrObjectPtr &object, int id, int lifetime, bool use_col_init) {
    visualization_msgs::Marker marker;
    marker.header = object->header;
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = object->sampledPoses.front().pose.position;
    marker.pose.orientation = object->sampledPoses.front().pose.orientation;

    if (use_col_init) {
        marker.color = createColorRGBA(0.0, 0.0, 1.0, 0.4);
        marker.ns = "Available objects";
    } else {
        marker.color = getMeshColor(object->identifier);
        marker.ns = "Recognition results";
    }

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.mesh_use_embedded_materials = true;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = object->meshResourcePath;
    marker.lifetime = ros::Duration(lifetime);

    return marker;
}


/* only working because the shape-based recognizer sets the observedId with the object color */
std_msgs::ColorRGBA FakeObjectRecognition::getMeshColor(std::string observed_id)
{
    std_msgs::ColorRGBA retColor = FakeObjectRecognition::createColorRGBA(0.0, 0.0, 0.0, 0.0);

    if (( observed_id.length() == 12 ) && ( observed_id.find_first_not_of("0123456789") == std::string::npos )) {
        float rgba[4];
        bool isColor = true;
        try {
            for (int i = 0; i <= 3; i++) {
                std::string temp;
                temp = observed_id.substr( (i * 3), 3 );
                rgba[i] = std::stof(temp) / 100.0;
            }
        } catch (std::invalid_argument& ia) {
            ROS_DEBUG_STREAM(ia.what());
            isColor = false;
        }

        if(isColor) {
            retColor = FakeObjectRecognition::createColorRGBA(rgba[0], rgba[1], rgba[2], 1.0);
        }
    }
    return retColor;
}

std_msgs::ColorRGBA FakeObjectRecognition::createColorRGBA(float red, float green, float blue, float alpha) {
    std_msgs::ColorRGBA color;

    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = alpha;

    return color;
}

std::vector<geometry_msgs::Point> FakeObjectRecognition::transformPoints(std::vector<geometry_msgs::Point> points_list, Eigen::Quaterniond rotation, Eigen::Vector3d translation) {
    std::vector<geometry_msgs::Point> result_list = points_list;
    rotation.normalize();
    Eigen::Matrix3d rot_mat = rotation.toRotationMatrix();
    for (unsigned int i = 0; i < points_list.size(); i++) {
        Eigen::Vector3d current_point = Eigen::Vector3d(points_list.at(i).x, points_list.at(i).y, points_list.at(i).z);
        current_point = rot_mat * current_point;
        current_point += translation;
        result_list.at(i) = createPoint(current_point.x(), current_point.y(), current_point.z());
    }
    return result_list;
}

std::array<geometry_msgs::Point, 8> FakeObjectRecognition::transformPoints(std::array<geometry_msgs::Point, 8> points_list, Eigen::Quaterniond rotation, Eigen::Vector3d translation) {
    std::vector<geometry_msgs::Point> points_vector;
    for (unsigned int i = 0; i < points_list.size(); i++) {
        points_vector.push_back(points_list.at(i));
    }
    points_vector = transformPoints(points_vector, rotation, translation);
    std::array<geometry_msgs::Point, 8> result_array;
    for (unsigned int i = 0; i < result_array.size(); i++) {
        result_array.at(i) = points_vector.at(i);
    }
    return result_array;
}

visualization_msgs::MarkerArray::Ptr FakeObjectRecognition::createNormalMarker(const ObjectConfig &object, int id, int lifetime) {
    // Visualize normals (similar to next_best_view VisualizationsHelper.hpp, MarkerHelper.cpp:
    visualization_msgs::MarkerArray::Ptr objectNormalsMarkerArrayPtr;

    Eigen::Matrix<float, 4, 1> color = Eigen::Matrix<float, 4, 1>(1.0, 1.0, 0.0, 1.0);
    Eigen::Matrix<float, 3, 1> scale = Eigen::Matrix<float, 3, 1>(0.005, 0.01, 0.005);
    std::string ns = "ObjectNormals";

    // create common arrow marker:
    visualization_msgs::Marker objectNormalMarker;
    objectNormalMarker.header.frame_id = "/map";
    objectNormalMarker.lifetime = ros::Duration(lifetime);
    objectNormalMarker.ns = ns;
    objectNormalMarker.action = visualization_msgs::Marker::ADD;

    objectNormalMarker.type = visualization_msgs::Marker::ARROW;
    objectNormalMarker.pose.position = createPoint(0, 0, 0);
    objectNormalMarker.pose.orientation.x = 0.0;
    objectNormalMarker.pose.orientation.y = 0.0;
    objectNormalMarker.pose.orientation.z = 0.0;
    objectNormalMarker.pose.orientation.w = 1.0;
    // the model size unit is mm
    objectNormalMarker.scale.x = scale[0];
    objectNormalMarker.scale.y = scale[1];
    objectNormalMarker.scale.z = scale[2];

    objectNormalMarker.color.r = color[0];
    objectNormalMarker.color.g = color[1];
    objectNormalMarker.color.b = color[2];
    objectNormalMarker.color.a = color[3];

    objectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
    // transform normals into world frame
    std::vector<geometry_msgs::Point> normals = normals_[object.getType()];
    geometry_msgs::Pose object_pose = object.getPose();
    Eigen::Quaterniond rotation(object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z);
    normals = transformPoints(normals, rotation, Eigen::Vector3d(0.0, 0.0, 0.0));
    for(unsigned int i = 0; i < normals_[object.getType()].size(); i++) {
        geometry_msgs::Point start = object.getPose().position;
        geometry_msgs::Point end;
        end = createPoint(0.07 * normals.at(i).x, 0.07 * normals.at(i).y, 0.07 * normals.at(i).z);
        end.x += start.x;
        end.y += start.y;
        end.z += start.z;
        // Set individual parts of objectNormalMarker:
        objectNormalMarker.header.stamp = ros::Time();
        objectNormalMarker.id = id + i;

        objectNormalMarker.points = std::vector<geometry_msgs::Point>(); // Reset objectNormalMarker.points
        objectNormalMarker.points.push_back(start);
        objectNormalMarker.points.push_back(end);

        objectNormalsMarkerArrayPtr->markers.push_back(objectNormalMarker);
    }
    return objectNormalsMarkerArrayPtr;
}

std::vector<geometry_msgs::Point> FakeObjectRecognition::getNormals(const ObjectConfig &object) {
    // init normals: (similar to next_best_view ObjectHelper.h)
    std::vector<geometry_msgs::Point> temp_normals = std::vector<geometry_msgs::Point>();

    // Takes the mesh file path and cuts off the beginning ("package://object_database/rsc/database/")
    // and everything after the next "_", leaving only the recognizer name of the object.
    std::vector<std::string> strvec;
    std::string in = object.getMeshName();
    boost::algorithm::trim_if(in, boost::algorithm::is_any_of("_/"));
    boost::algorithm::split(strvec, in, boost::algorithm::is_any_of("_/"));
    ROS_DEBUG_STREAM("Recognizer name of object: " << strvec.at(6));
    std::string recognizer = strvec.at(6);

    // Get the object's meta data containing the normals:
    object_database::ObjectMetaData objectMetaData;
    objectMetaData.request.object_type = object.getType();
    objectMetaData.request.recognizer = recognizer;

    if (!object_metadata_service_client_.exists()) { ROS_DEBUG_STREAM("/object_database/object_meta_data service is not available"); }
    else {
        object_metadata_service_client_.call(objectMetaData);
        if (!objectMetaData.response.is_valid) { ROS_DEBUG_STREAM("objectMetadata response is not valid for object type " << object.getType()); }
        else { temp_normals = objectMetaData.response.normal_vectors; }
    }
    return temp_normals; // temp_normals: Empty if no normals were found (if some objects don't have any).
}

bool FakeObjectRecognition::getBBfromFile(std::array<geometry_msgs::Point, 8> &result, std::string object_type) {
    // init bounding box:
     // open config/bounding_box_corners.xml and check whether the corners for the respective object have already been calculated and stored there:
     ROS_DEBUG_STREAM("Looking for bounding box corner points in " << bb_corners_file_name_);
     std::string corners_string;
     try {
         rapidxml::file<> xmlFile(bb_corners_file_name_.c_str());
         rapidxml::xml_document<> doc;
         doc.parse<0>(xmlFile.data());
         rapidxml::xml_node<> *root_node = doc.first_node();
         if (root_node) {
             rapidxml::xml_node<> *child_node = root_node->first_node();
             while (child_node) {
                rapidxml::xml_attribute<> *type_attribute = child_node->first_attribute("type");
                if (type_attribute) {
                    if (object_type == type_attribute->value()) {
                        rapidxml::xml_node<> *bb_corners_node = child_node->first_node();
                        if (bb_corners_node) {
                            corners_string = bb_corners_node->value();
                        }
                        else {
                            ROS_DEBUG_STREAM("Could not find values in node with attribute type = \"" << object_type << "\"");
                        }
                        break;
                    }
                }
                child_node = child_node->next_sibling();
            }
         }
     } catch(std::runtime_error err) {
         ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
     } catch (rapidxml::parse_error err) {
         ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
     }
     std::array<geometry_msgs::Point, 8> corner_points;
     if (corners_string.length() > 0) { // Bounding box corners were found
         // get the floats from the input file:
         std::stringstream cornerstream;
         cornerstream.str(corners_string);
         std::string corner_coord;
         std::vector<float> coord_list;
         try {
             while (std::getline(cornerstream, corner_coord, ' ')) {
                 coord_list.push_back(std::stof(corner_coord));
             }
         } catch (std::invalid_argument& ia) {
             ROS_DEBUG_STREAM(ia.what());
             coord_list = std::vector<float>(); // return empty list: try to calculate new corner points instead
         }
         unsigned int j = 0;
         for (unsigned int i = 0; i <= coord_list.size() - 3; i+=3) {
             corner_points.at(j) = createPoint(coord_list.at(i), coord_list.at(i+1), coord_list.at(i+2));
             j++;
         }
     }
     else {
         return false;
     }
     result = corner_points;
     return true;
}

std::array<geometry_msgs::Point, 8> FakeObjectRecognition::calculateBB(const ObjectConfig &object) {
    /* Takes a .dae file, parses it, and finds position vertices of the mesh if available.
    * Uses the vertices as input for ApproxMVBB, which calculates an approximated Minimum Volume Bounding Box.
    * Then the 8 corner points of the bouding box are calculated and transformed into the object frame.
    * Finally, they are scaled with 0.001 so the measurements are in m.
    * The points appear in the list in the following order (like in pbd_msgs/PbdObject.msg):
    *     4-----5          z
    *    /|    /|         /              x right
    *   / |   / |        /               y down
    *  0-----1  |       /-------x        z forwar
    *  |  |  |  |       |
    *  |  6--|--7       |
    *  | /   | /        |
    *  |/	  |/         y
    *  2-----3
    * If no bounding box could be found, a vector containing only the object's center 8 times is used.
    * Result is written to file bb_corners_file_name_.*/
    // Get mesh input file:
    std::string mesh_path = ros::package::getPath("object_database") + object.getMeshName().substr(25); // cuts off "package://object_database" from object's meshName
    ROS_DEBUG_STREAM("Looking for mesh in: " << mesh_path);

    // Parse the input file:
    std::string vertices;
    try {
        rapidxml::file<> xmlFile(mesh_path.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        rapidxml::xml_node<> *collada_node = doc.first_node("COLLADA");
        if (!collada_node) ROS_DEBUG_STREAM("Could not find node " << "COLLADA");
        else {
            rapidxml::xml_node<> *lib_geom_node = collada_node->first_node("library_geometries");
            if (!lib_geom_node) ROS_DEBUG_STREAM("Could not find node " << "library_geometries");
            else {
                rapidxml::xml_node<> *geom_node = lib_geom_node->first_node("geometry");
                if (!geom_node) ROS_DEBUG_STREAM("Could not find node " << "geometry");
                else {
                    rapidxml::xml_node<> *mesh_node = geom_node->first_node("mesh");
                    if (!mesh_node) ROS_DEBUG_STREAM("Could not find node " << "mesh");
                    else {
                        rapidxml::xml_node<> *source_node = mesh_node->first_node("source");
                        while (source_node) {
                            rapidxml::xml_attribute<> *name_attribute = source_node->first_attribute("name");
                            std::string name;
                            if (name_attribute) {
                                name = name_attribute->value();
                                if (name == "position") {
                                    rapidxml::xml_node<> *f_array_node = source_node->first_node("float_array");
                                    if (!f_array_node) ROS_DEBUG_STREAM("Could not find node " << "float_array");
                                    else {
                                        rapidxml::xml_node<> *array_node = f_array_node->first_node();
                                        if (!array_node) ROS_DEBUG_STREAM("Could not find node " << "containing the position array");
                                        else {
                                            vertices = array_node->value();
                                            break;
                                        }
                                    }
                                }
                            }
                            source_node = source_node->next_sibling("source");
                        }
                        if (!source_node) ROS_DEBUG_STREAM("Could not find node " << "source with name position and position array");
                    }
                }
            }
        }
    } catch(std::runtime_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
    } catch (rapidxml::parse_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
    }

    // get the floats from the input file's mesh-source node with name="positions":
    std::stringstream vertstream;
    vertstream.str(vertices);
    std::vector<float> vertex_list;
    std::string vertex_coord;
    float sum = 0.0;
    try {
        while (std::getline(vertstream, vertex_coord, ' ')) {
            float value= std::stof(vertex_coord);
            sum += std::abs(value);
            vertex_list.push_back(value);
        }
    } catch (std::invalid_argument& ia) {
            ROS_DEBUG_STREAM(ia.what());
            sum = 0.0; // set sum to 0 to invalidate all vertices found so far and use center point instead
    }

    // Set the minimum and maximum coordinates to 0 first
    float min_x, min_y, min_z, max_x, max_y, max_z;
    min_x = min_y = min_z = max_x = max_y = max_z = 0.0;

    ROS_DEBUG_STREAM("Calculating approximated oriented bounding box");
    ApproxMVBB::OOBB bb;

    if (sum > 0.0) { // not all vertices were 0. Otherwise the center of the object (0.0) is used instead of the bounding box.
        // Put the floats into Matrix for ApproxMVBB:
        ApproxMVBB::Matrix3Dyn points(3,vertex_list.size()/3);
        unsigned int j = 0;
        for (unsigned int i = 0; i <= vertex_list.size() - 3; i+=3) {
            points(0,j) = vertex_list.at(i);
            points(1,j) = vertex_list.at(i+1);
            points(2,j) = vertex_list.at(i+2);
                j++;
            }

            // Calculate bounding box:
            bb = ApproxMVBB::approximateMVBB(points,0.001,500,5,0,5);

            ApproxMVBB::Matrix33 A_KI = bb.m_q_KI.matrix().transpose();
            for(unsigned int i = 0;  i < points.cols(); ++i) {
                bb.unite(A_KI*points.col(i));
            }

            // Calculate all corner points in OOBB frame:
            ApproxMVBB::Vector3 min_point = bb.m_minPoint;
            ApproxMVBB::Vector3 max_point = bb.m_maxPoint;
            min_x = min_point.x();
            min_y = min_point.y();
            min_z = min_point.z();
            max_x = max_point.x();
            max_y = max_point.y();
            max_z = max_point.z();
    }

    // Set the corner points:
    std::array<ApproxMVBB::Vector3, 8> corners_amvbb;
    corners_amvbb.at(0) = ApproxMVBB::Vector3(min_x, min_y, min_z); // 0 - min_point
    corners_amvbb.at(1) = ApproxMVBB::Vector3(max_x, min_y, min_z);
    corners_amvbb.at(2) = ApproxMVBB::Vector3(min_x, max_y, min_z);
    corners_amvbb.at(3) = ApproxMVBB::Vector3(max_x, max_y, min_z);
    corners_amvbb.at(4) = ApproxMVBB::Vector3(min_x, min_y, max_z);
    corners_amvbb.at(5) = ApproxMVBB::Vector3(max_x, min_y, max_z);
    corners_amvbb.at(6) = ApproxMVBB::Vector3(min_x, max_y, max_z);
    corners_amvbb.at(7) = ApproxMVBB::Vector3(max_x, max_y, max_z); // 7 - max_point

    // Into object frame and scaled down with 0.001 (measures assumed to be in mm; to m):
    if (sum > 0.0) { // see above
        for (unsigned int i = 0; i < corners_amvbb.size(); i++) {
            corners_amvbb.at(i) = (bb.m_q_KI * corners_amvbb.at(i)) * 0.001;
        }
    }

    //Transform Vectors into geometry_msgs:
    std::array<geometry_msgs::Point, 8> corner_points;
    for (unsigned int i = 0; i < corners_amvbb.size(); i++) {
        corner_points.at(i) = createPoint(corners_amvbb.at(i).x(), corners_amvbb.at(i).y(), corners_amvbb.at(i).z());
    }

    // Writes points into config/bounding_box_corners.xml
    try {
        std::string corners_string;
        for (unsigned int i = 0; i < corner_points.size(); i++) {
            corners_string += boost::lexical_cast<std::string>(corner_points.at(i).x) + " " + boost::lexical_cast<std::string>(corner_points.at(i).y) + " "
                    + boost::lexical_cast<std::string>(corner_points.at(i).z) + " ";
        }
        std::string object_node = "<Object type=\"" + object.getType() + "\">" + corners_string + "</Object>";
        std::ifstream ifile;
        ifile.open(bb_corners_file_name_);
        if (ifile.is_open()) {
            std::string old_contents;
            std::string line;
            while(getline(ifile, line)) {
                old_contents.append(line);
            }
            ifile.close();
            if (old_contents.find("<Objects>") == 0) {
                std::ofstream ofile;
                ofile.open(bb_corners_file_name_);
                if (ofile.is_open()) {
                    std::string inner_nodes = old_contents.substr(9); // Cut off "<Objects>" in the beginning.
                    ofile << "<Objects>" << object_node << inner_nodes; // Reattach it and write new node behind it, followed by the rest of the old file
                    ofile.close();
                    ROS_DEBUG_STREAM("Bounding box corner points written to file " << bb_corners_file_name_);
                }
            }
            else ROS_DEBUG_STREAM("When trying to write bounding box corners to file " << bb_corners_file_name_ << ": Could not find root node " << "<Objects>" << ". Corners not written.");
        }
        else {
            ROS_DEBUG_STREAM("Could not open file " << bb_corners_file_name_ << ". Bounding box corners not written.");
        }
    } catch(boost::bad_lexical_cast err) {
        ROS_DEBUG_STREAM("Can't cast bounding box corner points to string. Cast error: " << err.what());
    }

    return corner_points;
}

geometry_msgs::Point FakeObjectRecognition::createPoint(double x, double y, double z) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}

}

int main(int argc, char** argv) {

    ros::init (argc, argv, "asr_fake_object_recognition");

    fake_object_recognition::FakeObjectRecognition rec;

    ros::spin();

    return 0;
}

