##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Osm2Dict() class
#             Output a list of roads and models that need to be simulated in
#             the gazebo form the data it recives from the .osm file
##############################################################################

import numpy as np


class Osm2Dict:

    def __init__(self, lonStart, latStart, data, flags=['a']):

        self.latStart = latStart
        self.lonStart = lonStart
        self.data = data
        self.displayAll = 'a' in flags
        self.displayModels = 'm' in flags
        self.displayRoads = 'r' in flags
        self.displayBuildings = "b" in flags
        self.flags = flags
        #Radius of the Earth
        self.R = 6371

        #Dictionaries to store results
        self.records = dict()
        self.models = dict()
        self.buildings = dict()
        #types of highways to be simulated
        self.highwayType = dict({"footway": 0.3, 'pedestrian': 3,
                                 "motorway": 14, "motorway_link": 13,
                                 "trunk": 12, "trunk_link": 11,
                                 "primary": 10, "primary_link": 9,
                                 "secondary": 8, "secondary_link": 7,
                                 "tertiary": 6, "tertiary_link": 5,
                                 "residential": 3,
                                 "steps": 0.8})

        #types of models and buildings to be simulated and a dictionary
        #associating them with models in gazebo and their occurences
        self.modelType = ['highway', 'amenity', 'building', 'emergency']

        self.addModel = dict({"stop": {"modelName": "stop_sign",
                                       "occurence": -1},
                              "street_lamp": {"modelName": "lamp_post",
                                              "occurence": -1},
                              "traffic_signals": {"modelName":
                                                  "construction_cone",
                                                  "occurence": -1},
                              "fire hydrant": {"modelName": "fire_hydrant",
                                               "occurence": -1},
                              "give_way": {"modelName": "speed_limit",
                                           "occurence": -1},
                              "bus_stop": {"modelName":
                                           "robocup14_spl_goal",
                                           "occurence": -1},
                              "fuel": {'modelName': "gas_station",
                                       'occurence': -1}
                              })

        self.amenityList = dict({"school": {"color": "Purple",
                                            "occurence": -1},
                                 "post_office": {'color': 'Orange',
                                                 'occurence': -1},
                                 "university": {"color": "Purple",
                                                'occurence': -1},
                                 "library": {"color": "Purple",
                                             'occurence': -1},
                                 "bar": {"color": "Blue",
                                         'occurence': -1},
                                 "cafe": {'color': "Blue",
                                          'occurence': -1},
                                 "pub": {"color": "Blue",
                                         'occurence': -1},
                                 "restaurant": {"color": "Blue",
                                                'occurence': -1},
                                 "fast_food": {"color": "Blue",
                                               'occurence': -1},
                                 "college": {"color": "Purple",
                                             'occurence': -1},
                                 "kindergarten": {"color": "Purple",
                                                  'occurence': -1}
                                 })

        self.node = {data[i].get("data").get("id"): data[i].get('data')
                     for i in range(len(data))
                     if data[i].get("type") == "node"}

        self.ways = [data[i].get('data')
                     for i in range(len(data))
                     if data[i].get("type") == "way"]

    def getPoints(self, coords):
        '''Input : latitude and longitudnal coordinates
           Return the points in gazebo frame with respect
           to the starting coordinates'''
        if not coords.any():
            return []

        lon2 = np.radians(coords[:, 0])
        lat2 = np.radians(coords[:, 1])

        dLat = lat2-np.radians(self.latStart)
        dLon = lon2-np.radians(self.lonStart)

        a = (np.sin(dLat/2) * np.sin(dLat/2) +
             np.sin(dLon/2) * np.sin(dLon/2) *
             np.cos(np.radians(self.latStart)) *
             np.cos(lat2))

        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

        distance = self.R * c

        angles = (np.arctan2(np.sin(dLon) * np.cos(lat2),
                  np.cos(np.radians(self.latStart)) *
                  np.sin(lat2) -
                  np.sin(np.radians(self.latStart)) *
                  np.cos(lat2) * np.cos(dLon)))

        point = np.array([distance*np.cos(angles) * 1000,
                          -distance*np.sin(angles) * 1000,
                          np.zeros(np.shape(distance))*1000])
        return point

    def latLonToPoints(self, node_ref):
        '''Pulls out the latitude and longitudes of the nodes in the
           list of nodes and gets the points in the gazebo frame'''
        coords = np.array([])
        for node in node_ref:

            coords = np.append(coords,
                               self.node[node]
                               .get("lon"))
            coords = np.append(coords,
                               self.node[node]
                               .get("lat"))
            coords = np.reshape(coords,
                                (len(coords)/2,
                                 2))

        pointsXYZ = self.getPoints(coords)
        return pointsXYZ

    def getRoadDetails(self):
        '''Returns a list of roads with corresponding widths'''
         # get the road latitude and longitudes
        for way in range(len(self.ways)):
            tagData = self.ways[way].get("tag")
            if "highway" in tagData:
                typeHighway = tagData.get("highway")

                if typeHighway in self.highwayType.keys():

                            roadName = tagData.get("name")

                            if roadName is None:
                                roadName = (typeHighway +
                                            "_" +
                                            str(self.ways[way]
                                                    .get("id")))
                            else:
                                roadName += "_" + str(self.ways[way]
                                                          .get("id"))

                            node_ref = self.ways[way].get("nd")
                            if node_ref:
                                location = self.latLonToPoints(node_ref)

                                self.records[roadName] = {'points':
                                                          location,
                                                          'width':
                                                          self.highwayType
                                                          [typeHighway],
                                                          'texture':
                                                          typeHighway.capitalize()}
        return self.records

    def getModelDetails(self):
        '''Returns a list of models to be included in the map'''
        models = {element + "$" + str(i): self.data[i].get("data")
                  for i in range(len(self.data))
                  for element in self.addModel.keys()
                  if element in self.data[i].get("data").get("tag").values()}

        for mName, data in models.iteritems():
            modelType = mName.split("$")[0]

            coords = np.array([data.get("lon"),
                               data.get("lat")])
            coords = np.reshape(coords, (len(coords)/2, 2))

            modelLocation = self.getPoints(coords)

            self.addModel[modelType]['occurence'] += 1

            repNum = self.addModel[modelType]['occurence']

            self.models[self.addModel
                        [modelType]
                        ['modelName']
                        + "_" +
                        str(repNum)] = {"points": modelLocation,
                                        "mainModel": self.addModel
                                        [modelType]['modelName']}

    def getBuildingDetails(self):
        '''Returns a list of buildings to be included in the map'''
        building = [self.data[i].get("data")
                    for i in range(len(self.data))
                    if "building" in self.data[i].get("data").get("tag")]

        for element in building:
            tagData = element.get("tag")
            if "name" in tagData:
                buildingName = tagData.get("name")
            else:
                buildingName = ("office_building" +
                                "_" +
                                str(element.get("id")))
            if "name_1" in tagData:
                buildingName += tagData.get("name_1")

            node_ref = element.get("nd")

            if node_ref:
                location = self.latLonToPoints(node_ref)

                buildingLoc = np.array([[sum(location[0, :]) /
                                         len(location[0, :])],
                                        [sum(location[1, :]) /
                                         len(location[1, :])],
                                        [sum(location[2, :]) /
                                         len(location[2, :])]]
                                       )

                self.buildings[buildingName] = {"mean":
                                                buildingLoc,
                                                "points": location,
                                                "color": "Red"}

        amenity = [self.data[i].get("data")
                   for i in range(len(self.data))
                   if "amenity" in self.data[i].get("data").get("tag")]

        for element in amenity:
            tagData = element.get("tag")
            if tagData.get("amenity") in self.amenityList.keys():

                amenity = tagData.get("amenity")

                node_ref = element.get("nd")
                if node_ref:
                    location = self.latLonToPoints(node_ref)

                    amenityLocation = np.array([[sum(location[0, :]) /
                                                 len(location[0, :])],
                                                [sum(location[1, :]) /
                                                 len(location[1, :])],
                                                [sum(location[2, :]) /
                                                 len(location[2, :])]]
                                               )

                    self.amenityList[amenity]['occurence'] += 1
                    repNum = self.amenityList[amenity]['occurence']

                    self.buildings[amenity +
                                   "_" +
                                   str(repNum)] = {"mean": amenityLocation,
                                                   "points": location,
                                                   "color":
                                                   self.amenityList
                                                   [amenity]
                                                   ['color']
                                                   }
        service = [self.data[i].get("data")
                   for i in range(len(self.data))
                   if "service" in self.data[i].get("data").get("tag")]

        for element in service:
            if element.get("tag").get("service") == "parking_aisle":
                node_ref = element.get("nd")
                if node_ref:
                    location = self.latLonToPoints(node_ref)

                    parkingLocation = np.array([[sum(location[0, :]) /
                                                 len(location[0, :])],
                                                [sum(location[1, :]) /
                                                 len(location[1, :])],
                                                [sum(location[2, :]) /
                                                 len(location[2, :])]]
                                               )

                    self.buildings["parking_aisle_" +
                                   str(element
                                       .get("id"))] = {"mean":
                                                       parkingLocation,
                                                       "points": location,
                                                       "color": "Yellow"
                                                       }

    def getMapDetails(self):
        ''' Returns a list of highways with corresponding widths
            and a list of all the models to be included'''
        if 'm' in self.flags or 'a' in self.flags:
            self.getModelDetails()

        if 'b' in self.flags or 'a' in self.flags:
            self.getBuildingDetails()

        if 'r' in self.flags or 'a' in self.flags:
            self.getRoadDetails()

        return self.records, self.models, self.buildings

    def setFlags(self, addFlag):
        '''sets the value for the list of flags'''
        if addFlag in ['a', 'm', 'r', 'b']:
            if addFlag not in self.flags:
                if addFlag == 'a':
                    self.flags.append(addFlag)
                else:
                    self.flags = [addFlag]
            return True
        else:
            print 'Error: Invalid flag! [Valid values : "a", "m", "r", "b"]'
            return False

    def getFlags(self):
        '''Returns the list of flags activated'''
        return self.flags

    def getLat(self):
        '''Get the latitude of the start point'''
        return self.latStart

    def getLon(self):
        '''Get the longitude of the start point'''
        return self.lonStart
