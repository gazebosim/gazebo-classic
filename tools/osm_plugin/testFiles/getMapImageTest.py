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
# Description: Unit test for getMapImage()
#              Uses the data from .osm file to output an image of the area
#              indicated in the .osm file and
#              Stores it in file with the specified name(.png)

import unittest
import sys
sys.path.insert(0, '../source')
from getMapImage import getMapImage


class MapImageTest(unittest.TestCase):

    def testPass(self):
        '''tests if the .png file opens correctly'''
        self.assertEqual(getMapImage('map.osm', 'map.png'), 0)

    def testFail(self):
        '''tests if no names for input and output files are specifed'''
        self.assertEqual(getMapImage('', ''), -1)

if __name__ == '__main__':
    unittest.main()
