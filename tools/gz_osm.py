#!/usr/bin/env python
import sys
sys.path.insert(0, 'osm_plugin/source')
import os
from lxml import etree
import argparse
from dict2sdf import GetSDF
from osm2dict import Osm2Dict
from getMapImage import getMapImage
from getOsmFile import getOsmFile

TIMER = 1


def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()


def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print ("Elapsed time is " + str(time.time()
               - startTime_for_tictoc)
               + " seconds.")
    else:
        print "Toc: start time not set"


parser = argparse.ArgumentParser()
parser.add_argument('-f', '--outFile',
                    help='Output file name', type=str, default='outFile.sdf')
parser.add_argument('-o', '--osmFile', help='Name of the osm file generated',
                    type=str,
                    default='map.osm')
parser.add_argument('-O', '--inputOsmFile', help='Name of the Input osm file',
                    type=str,
                    default='')
parser.add_argument('-i', '--imageFile',
                    help='Generate and name .png image of the selected areas',
                    type=str,
                    default='')
parser.add_argument('-d', '--directory',
                    help='Output directory',
                    type=str,
                    default='./')
parser.add_argument('-B', '--boundingbox',
                    help=('Give the bounding box for the area\n' +
                          'Format: MinLon MinLat MaxLon MaxLat'),
                    nargs='*',
                    type=float,
                    default=[-75.380, 40.606, -75.377, 40.609])

parser.add_argument('-r', '--roads',
                    help='Display Roads',
                    action='store_true')

parser.add_argument('-m', '--models',
                    help='Display models',
                    action='store_true')

parser.add_argument('-b', '--buildings',
                    help='Display buildings',
                    action='store_true')

parser.add_argument('-a', '--displayAll',
                    help='Display roads and models',
                    action='store_true')
parser.add_argument('--interactive',
                    help='Starts the interactive version of the program',
                    action='store_true')

args = parser.parse_args()

flags = []

if args.buildings:
    flags.append('b')

if args.models:
    flags.append('m')

if args.roads:
    flags.append('r')

if not(args.roads or args.models or args.buildings) or args.displayAll:
    flags.append('a')

if not os.path.exists(args.directory):
    os.makedirs(args.directory)

args.osmFile = args.directory + args.osmFile
args.outFile = args.directory + args.outFile

osmDictionary = {}

if args.interactive:
    print("\nPlease enter the latitudnal and logitudnal" +
          " coordinates of the area or select from" +
          " default by hitting return twice \n")

    startCoords = raw_input("Enter starting coordinates: " +
                            "[lon lat] :").split(' ')
    endCoords = raw_input("Enter ending coordnates: [lon lat]: ").split(' ')

    if (startCoords and endCoords and
            len(startCoords) == 2 and len(endCoords) == 2):

        for incoords in range(2):

            startCoords[incoords] = float(startCoords[incoords])
            endCoords[incoords] = float(endCoords[incoords])

    else:

        choice = raw_input("Default Coordinate options: West El " +
                           "Camino Real Highway, CA (2), Bethlehem," +
                           " PA (default=1): ")

        if choice != '2':
            startCoords = [40.61, -75.382]
            endCoords = [40.608, -75.3714]

        else:
            startCoords = [37.385844, -122.101464]
            endCoords = [37.395664, -122.083697]

    option = raw_input("Do you want to view the area specified? [Y/N]" +
                       " (default: Y): ").upper()

    osmFile = 'map.osm'
    args.boundingbox = [min(startCoords[1], endCoords[1]),
                        min(startCoords[0], endCoords[0]),
                        max(startCoords[1], endCoords[1]),
                        max(startCoords[0], endCoords[0])]

    if option != 'N':
        args.imageFile = 'map.png'

if args.inputOsmFile:
    f = open(args.inputOsmFile, 'r')
    root = etree.fromstring(f.read())
    f.close()
    args.boundingbox = [root[0].get('minlon'),
                        root[0].get('minlat'),
                        root[0].get('maxlon'),
                        root[0].get('maxlat')]
if TIMER:
    tic()
print "Downloading the osm data ... "
osmDictionary = getOsmFile(args.boundingbox,
                           args.osmFile, args.inputOsmFile)
if TIMER:
    toc()

if args.imageFile:
    if TIMER:
        tic()
    print "Building the image file ..."
    args.imageFile = args.directory + args.imageFile
    getMapImage(args.osmFile, args.imageFile)
    if TIMER:
        toc()

#Initialize the class
if TIMER:
    tic()
osmRoads = Osm2Dict(args.boundingbox[0], args.boundingbox[1],
                    osmDictionary, flags)

print "Extracting the map data for gazebo ..."
#get Road and model details
roadPointWidthMap, modelPoseMap, buildingLocationMap = osmRoads.getMapDetails()
if TIMER:
    toc()
if TIMER:
    tic()
print "Building sdf file ..."
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
    sdfFile.addRoad(road, roadPointWidthMap[road]['texture'])
    sdfFile.setRoadWidth(roadPointWidthMap[road]['width'], road)
    points = roadPointWidthMap[road]['points']
    for point in range(len(points[0, :])):
        sdfFile.addRoadPoint([points[0, point],
                              points[1, point],
                              points[2, point]],
                             road)

#output sdf File
sdfFile.writeToFile(args.outFile)
if TIMER:
    toc()
