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

namespace fake_object_recognition {

FakeObjectRecognition::FakeObjectRecognition() : nh_(NODE_NAME), config_changed_(false), recognition_released_(true) {
    ROS_DEBUG("Initialize process");
    nh_.getParam("fovx", fovx_);
    nh_.getParam("fovy", fovy_);
    nh_.getParam("ncp", ncp_);
    nh_.getParam("fcp", fcp_);
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

    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &FakeObjectRecognition::timerCallback, this);

    //Only used here to visualize object constellation. The rest (errors etc) is done at first doRecognition() call.
    loadObjects();

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

    std::string recognizable_objects;
    std::string objects_to_rec;
    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
        recognizable_objects += iter->getType() + " ";
    }
    ROS_DEBUG_STREAM("All recognizable objects: " << recognizable_objects);

    for (std::vector<std::string>::iterator iter = objects_to_rec_.begin(); iter != objects_to_rec_.end(); ++iter) {
        objects_to_rec += *iter + " ";
    }
    ROS_DEBUG_STREAM("Objects to recognize: " << objects_to_rec);

    for (std::vector<ObjectConfig>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
        if(std::find(objects_to_rec_.begin(), objects_to_rec_.end(), iter->getType()) == objects_to_rec_.end()) {
            continue;
        }
        ROS_INFO_STREAM("Current object: '" << (*iter).getType() << "'");
        geometry_msgs::Pose pose_left;
        geometry_msgs::Pose pose_right;
        try {
            if (config_.use_camera_pose) {
                ROS_DEBUG_STREAM("Transform into camera frame");

                pose_left = transformFrame((*iter).getPose(), frame_world_, frame_camera_left_);
                pose_right = transformFrame((*iter).getPose(), frame_world_, frame_camera_right_);

                ROS_DEBUG_STREAM("Pose left: " << pose_left.position.x << " " << pose_left.position.y << " " << pose_left.position.z << " " << pose_left.orientation.w << " " << pose_left.orientation.x << " " << pose_left.orientation.y << " " << pose_left.orientation.z << " ");
                ROS_DEBUG_STREAM("Pose right: " << pose_right.position.x << " " << pose_right.position.y << " " << pose_right.position.z << " " << pose_right.orientation.w << " " << pose_right.orientation.x << " " << pose_right.orientation.y << " " << pose_right.orientation.z << " ");

                pose_left = err_sim_.addNoiseToOrientation(err_sim_.addNoiseToPosition(pose_left));
                pose_right = err_sim_.addNoiseToOrientation(err_sim_.addNoiseToPosition(pose_right));

                ROS_DEBUG_STREAM("Pose left (with errors): " << pose_left.position.x << " " << pose_left.position.y << " " << pose_left.position.z << " " << pose_left.orientation.w << " " << pose_left.orientation.x << " " << pose_left.orientation.y << " " << pose_left.orientation.z << " ");
                ROS_DEBUG_STREAM("Pose right (with errors): " << pose_right.position.x << " " << pose_right.position.y << " " << pose_right.position.z << " " << pose_right.orientation.w << " " << pose_right.orientation.x << " " << pose_right.orientation.y << " " << pose_right.orientation.z << " ");
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

        if (!(config_.use_camera_pose) || objectIsVisible(pose_left, pose_right)) {
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

    //TODO: create bounding box
    boost::array< ::geometry_msgs::Point_<std::allocator<void> > , 8> bounding_box;
    for (unsigned int z = 0; z < 2; z++) {
        for (unsigned int y = 0; y < 2; y++) {
            for (unsigned int x = 0; x < 2; x++) {
                bounding_box[4 * z + 2 * y + x].x = 0.0;
                bounding_box[4 * z + 2 * y + x].y = 0.0;
                bounding_box[4 * z + 2 * y + x].z = 0.0;
            }
        }
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

}

int main(int argc, char** argv) {

    ros::init (argc, argv, "asr_fake_object_recognition");

    fake_object_recognition::FakeObjectRecognition rec;

    ros::spin();

    return 0;
}

