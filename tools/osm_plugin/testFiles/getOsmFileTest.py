#!/usr/bin/env python
##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Unit test for getOsmFile()
#             Downloads the .osm file for the stated bounding box and
#             Stores it in file with the specified name
##############################################################################

import unittest
import sys
sys.path.insert(0, '../source')
from getOsmFile import getOsmFile


class OsmFileTest(unittest.TestCase):

    def setUp(self):
        self.boxValid = [-75.385, 40.608, -75.378, 40.610]
        self.boxEmpty = []
        self.emptyFileName = ''
        self.fileName = 'map.osm'
        self.dataDict = {}

    def testBox(self):
        '''test to check if the function works for a correct bounding box '''
        self.assertNotEqual(getOsmFile(self.boxValid,
                                       self.fileName), {})
        self.dataDict.clear()

    def testEmptyBox(self):
        '''test to check the output, if the bounding box is empty'''
        self.assertEqual(getOsmFile(self.boxEmpty,
                                    self.fileName), None)
        self.dataDict.clear()

    def testInputFile(self):
        '''tests to check the output, if a osm file is directly given'''
        self.assertNotEqual(getOsmFile(self.boxEmpty,
                                       self.emptyFileName,
                                       self.fileName), None)
        self.dataDict.clear()


if __name__ == '__main__':
    unittest.main()
