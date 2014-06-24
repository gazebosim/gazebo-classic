##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: GetSDF() class
#             Builds a sdf file by adding models and seting their properties,
#             roads and sets spherical coordinates for the world
##############################################################################

import lxml.etree as Et
#import xml.dom.minidom as minidom
import numpy


class GetSDF:

    def __init__(self):
        self.sdf = Et.Element('sdf')
        self.sdf.set('version', "1.4")
        world = Et.SubElement(self.sdf, 'world')
        world.set('name', 'default')
        self.modelList = dict()

    def addSphericalCoords(self, latVal, lonVal,
                           elevationVal=0.0, headingVal=0):
        ''' Add the spherical coordinates for the map'''
        spherical_coordinates = Et.SubElement(self.sdf.find('world'),
                                              'spherical_coordinates')

        model = Et.SubElement(spherical_coordinates, 'surface_model')
        model.text = "EARTH_WGS84"

        lat = Et.SubElement(spherical_coordinates, 'latitude_deg')
        lat.text = str(latVal)

        lon = Et.SubElement(spherical_coordinates, 'longitude_deg')
        lon.text = str(lonVal)

        elevation = Et.SubElement(spherical_coordinates, 'elevation')
        elevation.text = str(elevationVal)

        heading = Et.SubElement(spherical_coordinates, 'heading_deg')
        heading.text = str(headingVal)

    def includeModel(self, modelName):
        ''' Include models in gazebo database'''
        includeModel = Et.SubElement(self.sdf.find('world'), 'include')
        includeUri = Et.SubElement(includeModel, 'uri')
        includeUri.text = "model://" + modelName
        return includeModel

    def addModel(self, mainModel, modelName, pose):
        '''Add model with pose and the name taken as inputs'''

        includeModel = self.includeModel(mainModel)

        model = Et.SubElement(includeModel, 'name')
        model.text = modelName

        static = Et.SubElement(includeModel, 'static')
        static.text = 'true'

        modelPose = Et.SubElement(includeModel, 'pose')

        modelPose.text = (str(pose[0]) +
                          " " + str(pose[1]) +
                          " " + str(pose[2]) + " 0 0 0")

    def addRoad(self, roadName, roadType):
        '''Add road to sdf file'''
        road = Et.SubElement(self.sdf.find('world'), 'road')
        road.set('name', roadName)
        material = Et.SubElement(road, 'material')
        script = Et.SubElement(material, 'script')
        Et.SubElement(script, 'uri').text = ('file://media/materials/' +
                                             'scripts/gazebo.material')
        Et.SubElement(script, 'name').text = 'Gazebo/' + roadType

    def setRoadWidth(self, width, roadName):
        ''' Set the width of the road specified by the road name'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]

        roadWidth = Et.SubElement(roadWanted[0], 'width')
        roadWidth.text = str(width)

    def addRoadPoint(self, point, roadName):
        '''Add points required to build a road, specified by the roadname'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]
        roadPoint = Et.SubElement(roadWanted[0], 'point')
        roadPoint.text = (str(point[0]) +
                          " " + str(point[1]) +
                          " " + str(point[2]))

    def addBuilding(self, mean, pointList, building_name, color):
        building = Et.SubElement(self.sdf.find('world'), 'model')
        building.set('name', building_name)
        static = Et.SubElement(building, 'static')
        static.text = 'true'
        mainPose = Et.SubElement(building, 'pose')
        mainPose.text = (str(mean[0, 0]) +
                         " " + str(mean[1, 0]) +
                         " " + str(mean[2, 0]) +
                         " 0 0 0")

        yaw = [numpy.arctan2((pointList[1, point] - pointList[1, point + 1]),
                             (pointList[0, point] - pointList[0, point + 1]))
               for point in range(numpy.size(pointList, 1)-1)]

        distance = [numpy.sqrt(((pointList[1, point] -
                                 pointList[1, point + 1])**2 +
                                (pointList[0, point] -
                                 pointList[0, point + 1])**2))
                    for point in range(numpy.size(pointList, 1)-1)]

        meanPoint = [[(pointList[0, point] +
                      pointList[0, point + 1])/2 - mean[0, 0],
                     (pointList[1, point] +
                      pointList[1, point + 1])/2 - mean[1, 0], 0]
                     for point in range(numpy.size(pointList, 1)-1)]

        for point in range(len(yaw)):

            link = Et.SubElement(building, 'link')
            link.set('name', (building_name + '_' + str(point)))
            collision = Et.SubElement(link, 'collision')
            collision.set('name', (building_name + '_' + str(point)))

            geometry = Et.SubElement(collision, 'geometry')
            box = Et.SubElement(geometry, 'box')
            Et.SubElement(box, 'size').text = str(distance[point]) + ' 0.2 0'

            visual = Et.SubElement(link, 'visual')
            visual.set('name', (building_name + '_' + str(point)))

            geometry = Et.SubElement(visual, 'geometry')
            box = Et.SubElement(geometry, 'box')
            Et.SubElement(box, 'size').text = str(distance[point]) + ' 0.2 0'

            material = Et.SubElement(visual, 'material')
            script = Et.SubElement(material, 'script')
            Et.SubElement(script, 'uri').text = ('file://media/materials/' +
                                                 'scripts/gazebo.material')
            Et.SubElement(script, 'name').text = 'Gazebo/' + color
            Et.SubElement(link, 'pose').text = (str(meanPoint[point][0]) +
                                                ' ' +
                                                str(meanPoint[point][1]) +
                                                ' 0 0 0 ' +
                                                str(yaw[point]))

    def writeToFile(self, filename):
        '''Write sdf file'''
        outfile = open(filename, "w")
        outfile.write(Et.tostring(self.sdf, pretty_print=True,
                                  xml_declaration=True))
        outfile.close()
