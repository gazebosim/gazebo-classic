#!/usr/bin/env python
##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Unit test for getMapImage()
#             Uses the data from .osm file to output an image of the area
#             indicated in the .osm file and
#             Stores it in file with the specified name(.png)
##############################################################################

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
