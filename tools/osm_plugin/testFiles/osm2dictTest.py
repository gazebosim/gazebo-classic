#!/usr/bin/env python
##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Unit test for Osm2Dict() class
#             Output a list of roads and models that need to be simulated in
#             the gazebo form the data it recives from the .osm file
##############################################################################

import numpy as np
import unittest
import sys
sys.path.insert(0, '../source')

from osm2dict import Osm2Dict
from getOsmFile import getOsmFile


class Osm2DictTest(unittest.TestCase):

    def setUp(self):
        self.osmDict = {}
        self.osmDict = getOsmFile([-75.38, 40.606, -75.377, 40.609], 'map.osm')
        self.testClass = Osm2Dict(-75.93, 40.61, self.osmDict)

    def testPointsEmpty(self):
        '''tests if the getPoints() returns an empty list
           if it recives the same'''
        self.assertEqual(self.testClass.getPoints(np.array([])), [])

    def testPoints(self):
        '''tests the getPoints() function work'''
        self.assertEqual(self
                         .testClass
                         .getPoints(np.array([[-75.83, 41.61]])).all(),
                         np.array([[111200.57170516],
                                   [111183.63531948],
                                   [0.]]).all())

    def testNumRoadsModels(self):
        '''tests if the number of roads and models forund is correct'''
        roadList, modelsList, buildingList = self.testClass.getMapDetails()
        self.assertEqual(len(roadList.keys()), 95)
        self.assertEqual(len(modelsList.keys()), 5)
        self.assertEqual(len(buildingList.keys()), 23)

    def testSetGetFlags(self):
        '''tests if setFlags() and getFlags() methods work'''
        self.testClass.setFlags('m')
        self.assertEqual(self.testClass.getFlags(), ['m'])

    def testModels(self):
        '''tests if the models only option works'''
        self.testClass.setFlags('m')
        roadList, modelsList, buildingList = self.testClass.getMapDetails()
        self.assertEqual(len(roadList.keys()), 0)
        self.assertEqual(len(modelsList.keys()), 5)
        self.assertEqual(len(buildingList.keys()), 0)

    def testRoads(self):
        '''tests if the roads only option works'''
        self.testClass.setFlags('r')
        roadList, modelsList, buildingList = self.testClass.getMapDetails()
        self.assertEqual(len(roadList.keys()), 95)
        self.assertEqual(len(modelsList.keys()), 0)
        self.assertEqual(len(buildingList.keys()), 0)

    def testBuildings(self):
        '''tests if the roads only option works'''
        self.testClass.setFlags('b')
        roadList, modelsList, buildingList = self.testClass.getMapDetails()
        self.assertEqual(len(roadList.keys()), 0)
        self.assertEqual(len(modelsList.keys()), 0)
        self.assertEqual(len(buildingList.keys()), 23)


if __name__ == '__main__':
    unittest.main()
