# Copyright 2014 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Tashwin Khurana
# Description: getOsmFile()
#              Downloads the .osm file for the stated bounding box and
#              Stores it in file with the specified name

import urllib2
import osmapi


def getOsmFile(box, outputFile='map.osm', inputOsmFile=''):
    '''downloads the data file for the specified bounding box
       stores the file as outputFile, if inputOsmFile is not specified
       and also converts the data in the form of a dictionary'''
    if not box and not inputOsmFile:
        return None

    dataDict = {}
    if inputOsmFile:
        outputFile = inputOsmFile
    else:
        try:
            osmFile = urllib2.urlopen('http://api.openstreetmap.org' +
                                      '/api/0.6/map?bbox='
                                      + str(box)[1:-1].replace(" ", ""))
        except urllib2.HTTPError:
            print ("\nError:\tPlease check the bounding box input arguments"
                   + "\n\tFormat: MinLon MinLat MaxLon MaxLat")
            return {}
        osm = open(outputFile, 'w')

        osm.write(osmFile.read())

        osm.close()

    osmRead = open(outputFile, 'r')

    myapi = osmapi.OsmApi()

    dataDict = myapi.ParseOsm(osmRead.read())

    osmRead.close()

    return dataDict
