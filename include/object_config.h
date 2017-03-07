/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef OBJECT_CONFIG_H
#define OBJECT_CONFIG_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <object_config.h>


namespace fake_object_recognition {

/**
 * \brief This class is used to save information about an available object
 */
class ObjectConfig {

private:
    /** The name of the object **/
    std::string type_;

    /** The id of the object **/
    std::string id_;

    /** The pose of the object **/
    geometry_msgs::Pose pose_;

    /** The path to the mesh used to visualize the object **/
    std::string mesh_name_;

    /** The normals of the object **/
    std::vector<geometry_msgs::Point> normals_;

    /** The bounding box corner points of the object **/
    std::vector<geometry_msgs::Point> bb_corners_;


public:
    /**
     * \brief The constructor of this class
     *
     * \param type          The name of the object
     * \param id            The id of the object
     * \param pose          The pose of the object
     * \param mesh_name     The path to the object's mesh file
     */
    ObjectConfig(const std::string &type, const std::string id, const geometry_msgs::Pose &pose, const std::string &mesh_name);

    /**
     * \brief Return the name of the object
     *
     * @return The object's name
     */
    std::string getType() const;

    /**
     * \brief Return the id of the object
     *
     * @return The object's id
     */
    std::string getId() const;

    /**
     * \brief Return the pose of the object
     *
     * @return The object's pose
     */
    geometry_msgs::Pose getPose() const;

    /**
     * \brief Return the mesh path of the object
     *
     * @return The object's mesh path
     */
    std::string getMeshName() const;
    
};

}


#endif /* OBJECT_CONFIG_H */


