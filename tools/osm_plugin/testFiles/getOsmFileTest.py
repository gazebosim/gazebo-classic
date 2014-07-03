#!/usr/bin/env python
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
# Description: Unit test for getOsmFile()
#              Downloads the .osm file for the stated bounding box and
#              Stores it in file with the specified name

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
