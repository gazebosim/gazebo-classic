##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: getOsmFile()
#             Downloads the .osm file for the stated bounding box and
#             Stores it in file with the specified name
##############################################################################

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
