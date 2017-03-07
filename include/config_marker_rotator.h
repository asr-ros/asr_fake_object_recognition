/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef CONFIGMARKERROTATOR_H
#define CONFIGMARKERROTATOR_H

#include <string>
#include <vector>
#include <stdio.h>
#include <Eigen/Geometry>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
/**
 * Tool to rotate marker based objects in constellations configuration (xml files) to be adapted to the used coordinates system
 */
struct XmlObject {
    XmlObject(std::string type, std::string id, std::string mesh, std::string angles, std::string poseString);
    std::string type;
    std::string id;
    std::string mesh;
    std::string angles;
    std::string poseString;
};

typedef boost::shared_ptr<XmlObject> XmlObjectPtr;
class ConfigMarkerRotator
{
public:
    ConfigMarkerRotator();
    void loadConstellationFromConfigFile(const std::string& configFilePath);
    void rotateAnglesOfObject(XmlObjectPtr object, std::string delim);
    void writeConstellationToConfigFile(const std::string& configFilePath);


public:
    std::vector<XmlObjectPtr> xmlObjects;
};

#endif // CONFIGMARKERROTATOR_H
