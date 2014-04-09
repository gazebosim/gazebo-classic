##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: getMapImage()
#             Uses the data from .osm file to output an image of the area
#             indicated in the .osm file and
#             Stores it in file with the specified name(.png)
##############################################################################

import os
try:
    import mapnik
    HAS_MAPNIK = True
except ImportError:
    HAS_MAPNIK = False


def getMapImage(osmFile, map_output):
    '''Uses the data from the osmFile to out a .png image
       of the area depicted by the input file'''

    if not HAS_MAPNIK:
        print ('Error: Mapnik module is missing. ' +
               'Please install for getting the image functionality.')
        return -2

    if osmFile == '':
        print 'Error: getMapImage::No File Recieved'
        return -1

    highwaList = dict({"motorway": {'width': 4,
                                    'color': 'green',
                                    'fontSize': 12},
                       "trunk": {'width': 3,
                                 'color': 'green',
                                 'fontSize': 11},
                       "primary": {'width': 1.5,
                                   'color': '#0090ff',
                                   'fontSize': 10},
                       "secondary": {'width': 0.8,
                                     'color': '#ff00ff',
                                     'fontSize': 8},
                       "tertiary": {'width': 0.42,
                                    'color': '#000000',
                                    'fontSize': 8},
                       "residential": {'width': 0.21,
                                       'color': 'black',
                                       'fontSize': 8},
                       "living_street": {'width': 0.21,
                                         'color': 'black',
                                         'fontSize': 8},
                       "pedestrian": {'width': 0.21,
                                      'color': 'brown',
                                      'fontSize': 8},
                       "footway": {'width': 0.21,
                                   'color': 'brown',
                                   'fontSize': 8}})

    m = mapnik.Map(1024, 1024)
    m.background = mapnik.Color('white')

    for highwayType in highwaList.keys():
        styleType = mapnik.Style()

        rule = mapnik.Rule()

        rule.filter = mapnik.Expression('[highway]=' + "'" + highwayType + "'")

        stk = mapnik.Stroke()
        stk.color = mapnik.Color(highwaList[highwayType]['color'])
        stk.line_cap = mapnik.line_cap.ROUND_CAP
        stk.width = highwaList[highwayType]['width']

        line_symbolizer = mapnik.LineSymbolizer(stk)

        rule.symbols.append(line_symbolizer)

        rule2 = mapnik.Rule()

        rule2.filter = mapnik.Expression('[highway]=' + "'" +
                                         highwayType + "'")

        text_symbolizer = mapnik.TextSymbolizer(mapnik.Expression("[name]"),
                                                "DejaVu Sans Book",
                                                highwaList[highwayType]
                                                          ['fontSize'],
                                                mapnik.Color('black'))
        text_symbolizer.halo_fill = mapnik.Color('white')

        rule2.symbols.append(text_symbolizer)

        styleType.rules.append(rule)
        styleType.rules.append(rule2)

        m.append_style(highwayType, styleType)

    ds = mapnik.Osm(file=osmFile)

    layer = mapnik.Layer('world')
    layer.datasource = ds
    for highwayType in highwaList.keys():
        layer.styles.append(highwayType)

    m.layers.append(layer)
    m.zoom_all()
    mapnik.render_to_file(m, map_output, 'png')

    return os.system('xdg-open ' + map_output)
