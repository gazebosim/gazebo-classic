#!/usr/bin/env python

# Desc: Utility for generating model figures
# Author: Andrew Howard
# Date: 15 Aug 2004
# CVS: $Id: makefig.py,v 1.4 2004/11/17 00:44:25 inspectorg Exp $

import sys
#import gazebo
import os
    
viewpoints = ('0 0 -90', '0 0 180', '0 0 90', '0 0 0',
              '90 0 -90', '-90 0 -90',
              '-26 30 135')


def make_world(modelname, dist, view, solid):
    """Create a world file."""

    file = open('makefig.world', 'r')
    world_template = file.read()

    cam_pose = '<xyz>-%f 0 0</xyz>\n<rpy>0 0 0</rpy>\n<polygonFill>%d</polygonFill>' % (dist, solid)
    rot = '<rpy>%s</rpy>' % viewpoints[view]
    
    model_desc = '<model:%s> <id>foo</id> %s </model:%s>\n' % (modelname, rot, modelname)

    file = open('tmp.world', 'w+')
    file.write(world_template % (cam_pose, model_desc))
    return


def capture(modelname, view, solid):
    """Run the sim and capture an image."""

    # Run gazebo to grab picture
    if os.spawnlp(os.P_WAIT, 'gazebo', 'gazebo', '-t 1.0', 'tmp.world') != 0:
        print 'failed'
        return
    print
    

    # Grab the image
    imagename = 'html/figures/%s-%d-%d' % (modelname, view, solid)
    print 'converting...',; sys.stdout.flush()
    os.system('convert frames/0001.pnm %s.gif' % imagename)
    os.system('convert -antialias -resize 160x120 frames/0001.pnm %s-thumb.gif' % imagename)
    print 'done'

    return


def write_html(modelname):
    """Create an html include file to go with the figure."""

    # Generate oblique view
    file = open('html/figures/%s_view.html' % modelname, 'w+')
    imagename = 'figures/%s-%d-%d' % (modelname, 6, 1)
    file.write('<a href="%s.gif"><img width src="%s-thumb.gif"></a>\n' % (imagename, imagename))
    imagename = 'figures/%s-%d-%d' % (modelname, 6, 0)
    file.write('<a href="%s.gif"><img width src="%s-thumb.gif"></a>\n' % (imagename, imagename))
    file.close()

    # Generate all views
    file = open('html/figures/%s_more_views.html' % modelname, 'w+')

    file.write('<table>\n')

    file.write('<tr>\n')
    imagename = 'figures/%s-%d-1' % (modelname, 6)
    file.write('<td rowspan=3><a href="%s.gif"><img width src="%s-thumb.gif"><a></td>\n' % (imagename, imagename))
    imagename = 'figures/%s-%d-1' % (modelname, 4)
    file.write('<td><a href="%s.gif"><img src="%s-thumb.gif"></a></td>\n' % (imagename, imagename))
    file.write('</tr>\n')

    file.write('<tr>\n')
    for view in range(4):
        imagename = 'figures/%s-%d-1' % (modelname, view)
        file.write('<td><a href="%s.gif"><img src="%s-thumb.gif"></a></td>\n' % (imagename, imagename))
    file.write('</tr>\n')

    file.write('<tr>\n')
    imagename = 'figures/%s-%d-1' % (modelname, 5)
    file.write('<td><a href="%s.gif"><img src="%s-thumb.gif"></a></td>\n' % (imagename, imagename))
    file.write('</tr>\n')

    file.write('</table>\n')
    return
                



if __name__ == '__main__':

    modelname = sys.argv[1]
    dist = float(sys.argv[2])

    for view in range(len(viewpoints)):
        make_world(modelname, dist, view, 0)
        capture(modelname, view, 0)
        make_world(modelname, dist, view, 1)
        capture(modelname, view, 1)

    write_html(modelname)
