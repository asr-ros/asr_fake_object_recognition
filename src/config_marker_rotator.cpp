/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "config_marker_rotator.h"

using namespace std;

XmlObject::XmlObject(string type, string id, string mesh, string angles, string poseString) : type(type), id(id), mesh(mesh), angles(angles), poseString(poseString) {}

ConfigMarkerRotator::ConfigMarkerRotator() {
}

void ConfigMarkerRotator::loadConstellationFromConfigFile(const std::string &configFilePath) {
    std::string xml_path = configFilePath;
    cout << "Path to objects.xml: " << xml_path << endl;

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
                    string type = type_attribute->value();
                    string id = id_attribute->value();
                    string angle = angles_attribute->value();
                    string mesh = mesh_attribute->value();
                    string pose_string = child_node->value();
                    cout << "type " << type << ", id " << id << ", angles " << angle << ", mesh " << mesh << ", pose_string " << pose_string << endl;
                    xmlObjects.push_back(XmlObjectPtr(new XmlObject(type, id, mesh, angle, pose_string)));
                    cout << xmlObjects.size() << endl;
                }
                child_node = child_node->next_sibling();
            }
        }
    } catch(std::runtime_error err) {
        cout << "Can't parse xml-file. Runtime error: " << err.what() << endl;
    } catch (rapidxml::parse_error err) {
        cout << "Can't parse xml-file Parse error: " << err.what() << endl;
    }
}

void ConfigMarkerRotator::rotateAnglesOfObject(XmlObjectPtr object, string delim) {
    string angles = object->angles;
    string pose_string = object->poseString;
    std::vector<std::string> strvec;

    boost::algorithm::trim_if(pose_string, boost::algorithm::is_any_of(delim));
    boost::algorithm::split(strvec, pose_string, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
    if (strvec.size() == 6 || strvec.size() == 7 ) {
        try {
                for (unsigned i = 0; i < strvec.size(); i++) {
                    boost::trim(strvec[i]);
                }
                string x = strvec[0];
                string y = strvec[1];
                string z = strvec[2];

                if(angles == "quaternion" && strvec.size() == 7)
                {
                    double qW = boost::lexical_cast<double>(strvec[3]);
                    double qX = boost::lexical_cast<double>(strvec[4]);
                    double qY = boost::lexical_cast<double>(strvec[5]);
                    //boost::trim_right(strvec[6]);
                    double qZ = boost::lexical_cast<double>(strvec[6]);
                    Eigen::Quaterniond quat(qW, qX, qY, qZ);
                    Eigen::Matrix3d A = quat.toRotationMatrix();
                    Eigen::Matrix3d B;
                    B = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
                    Eigen::Matrix3d C = A * B;
                    Eigen::Quaterniond rotatedQuat(C);
                    object->poseString = x + ","
                            + y + ","
                            + z + ","
                            + boost::lexical_cast<std::string>(rotatedQuat.w()) + ","
                            + boost::lexical_cast<std::string>(rotatedQuat.x()) + ","
                            + boost::lexical_cast<std::string>(rotatedQuat.y()) + ","
                            + boost::lexical_cast<std::string>(rotatedQuat.z());
                    return;

                }
                else if(angles == "euler" && strvec.size() == 6)
                {
                    double qX = boost::lexical_cast<double>(strvec[3]) * M_PI/180;
                    double qY = boost::lexical_cast<double>(strvec[4]) * M_PI/180;
                    boost::trim_right(strvec[5]);
                    double qZ = boost::lexical_cast<double>(strvec[5]) * M_PI/180;
                    Eigen::Matrix3d rotMat;
                    rotMat = Eigen::AngleAxisd(qX, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(qY,  Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(qZ, Eigen::Vector3d::UnitZ());
                    Eigen::Matrix3d B;
                    B = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
                    Eigen::Matrix3d C = rotMat * B;
                    Eigen::Vector3d euler = C.eulerAngles(0,1,2);
                    object->poseString = x + ","
                            + y + ","
                            + z + ","
                            + boost::lexical_cast<std::string>(euler[0] *180/M_PI) + ","
                            + boost::lexical_cast<std::string>(euler[1] *180/M_PI) + ","
                            + boost::lexical_cast<std::string>(euler[2] *180/M_PI);
//                    Eigen::Quaterniond rotatedQuat(C);
//                    object->angles = "quaternion";
//                    object->poseString = x + ","
//                            + y + ","
//                            + z + ","
//                            + boost::lexical_cast<std::string>(rotatedQuat.w()) + ","
//                            + boost::lexical_cast<std::string>(rotatedQuat.x()) + ","
//                            + boost::lexical_cast<std::string>(rotatedQuat.y()) + ","
//                            + boost::lexical_cast<std::string>(rotatedQuat.z());
                    return;
                }
                return;
        } catch (boost::bad_lexical_cast err) {
           cout << "Can't cast node-value. Cast error: " << err.what() << endl;
            return;
        }
    }
}

void ConfigMarkerRotator::writeConstellationToConfigFile(const string &configFilePath) {
    std::ofstream myfile;
    std::ostringstream s;
    myfile.open (configFilePath);
    myfile << "<Objects>";
    for (XmlObjectPtr object : xmlObjects) {
        myfile << "<Object "
           << "type=\"" << object->type
           << "\" id=\"" << object->id
           << "\" mesh=\"" << object->mesh
           << "\" angles=\"" << object->angles << "\">"
           << object->poseString
           << " </Object>";

    }
    myfile << "</Objects>";
    myfile.close();
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "You must give 2 pathes:" << endl;
        cout << "    - 1st path is the source file" << endl;
        cout << "    - 2nd path is the target file" << endl;
        return 1;
    }
    const string& sourceFile(argv[1]);
    const string& targetFile(argv[2]);
    ConfigMarkerRotator *rotator = new ConfigMarkerRotator();
    rotator->loadConstellationFromConfigFile(sourceFile);
    for (XmlObjectPtr object : rotator->xmlObjects) {
        auto found = object->type.find("marker");
        if (found != std::string::npos)
            rotator->rotateAnglesOfObject(object, ",");
    }
    rotator->writeConstellationToConfigFile(targetFile);
    cout << "Finished.." << endl;
}
