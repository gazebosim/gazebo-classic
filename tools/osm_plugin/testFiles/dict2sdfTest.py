#!/usr/bin/env python
##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Unit test for GetSDF() class
#             Builds a sdf file by adding models and seting their properties,
#             roads and sets spherical coordinates for the world
##############################################################################

import unittest
from lxml import etree
import os
import sys
sys.path.insert(0, '../source')

from getOsmFile import getOsmFile
from osm2dict import Osm2Dict
from dict2sdf import GetSDF


class GetSDFTest(unittest.TestCase):

    def setUp(self):
        '''Build an sdf file from a known set of data'''
        osmDict = {}
        osmDict = getOsmFile([-75.93, 40.61, -75.90, 40.62], 'map.osm')
        osmRoads = Osm2Dict(-75.93, 40.61, osmDict)
        (roadPointWidthMap,
         modelPoseMap,
         buildingLocationMap) = osmRoads.getMapDetails()

        #Initialize the getSdf class
        sdfFile = GetSDF()

        #Set up the spherical coordinates
        sdfFile.addSphericalCoords(osmRoads.getLat(), osmRoads.getLon())

        #add Required models
        sdfFile.includeModel("sun")

        for model in modelPoseMap.keys():
            points = modelPoseMap[model]['points']
            sdfFile.addModel(modelPoseMap[model]['mainModel'],
                             model,
                             [points[0, 0], points[1, 0], points[2, 0]])

        for building in buildingLocationMap.keys():
            sdfFile.addBuilding(buildingLocationMap[building]['mean'],
                                buildingLocationMap[building]['points'],
                                building,
                                buildingLocationMap[building]['color'])

        #Include the roads in the map in sdf file
        for road in roadPointWidthMap.keys():
            sdfFile.addRoad(road, road[road]['texture'])
            sdfFile.setRoadWidth(roadPointWidthMap[road]['width'], road)
            points = roadPointWidthMap[road]['points']
            for point in range(len(points[0, :])):
                sdfFile.addRoadPoint([points[0, point],
                                      points[1, point],
                                      points[2, point]],
                                     road)
        #output sdf File
        sdfFile.writeToFile('outFile.sdf')

    def validateSchema(self):
        '''function to test whether the output file
           conforms to the defined schema'''
        try:
            with open('outFile.sdf', 'r') as f:
                etree.fromstring(f.read(),
                                 base_url=
                                 "http://sdformat.org/schemas/world.xsd")
            return True
        except:
            return False

    def gzCheck(self):
        '''runs the gzsdf check command on the output file'''
        return os.system('gzsdf check outFile.sdf')

    def testXMLSchema(self):
        '''tests whether the output file
           conforms to the defined schema'''
        self.assertTrue(self.validateSchema())

    def testGzSDF(self):
        '''tests the file usin the gzsdf check command'''
        self.assertEqual(self.gzCheck(), 0)


if __name__ == '__main__':
    unittest.main()
