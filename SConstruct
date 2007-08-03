import os
import sys
import time 
version = '0.8'

#
# Setup the Options
#
opts = Options()
opts.Add('prefix', 'The install path "prefix"', '/usr/local')
opts.Add('destdir', 'The root directory to install into. Useful mainly for binary package building', '/')

#
# Function used to test for ODE library, and TriMesh suppor
#
ode_test_source_file = """
#include <ode/ode.h>
int main()
{
  dGeomTriMeshDataCreate();
  return 0;
}
"""

def CheckODELib(context):
  context.Message('Checking for ODE...')
  oldLibs = context.env['LIBS']
  context.env.Replace(LIBS='ode')
  result = context.TryLink(ode_test_source_file, '.cpp')
  context.Result(result)
  context.env.Replace(LIBS=oldLibs)
  return result

#
# Create the pkg-config file
#
def createPkgConfig(target, source, env):
  f = open(str(target[0]), 'wb')
  prefix = source[0].get_contents()
  f.write('prefix=' + prefix + '\n')
  f.write('Name: gazebo\n')
  f.write('Description: Simplified interface to Player\n')
  f.write('Version:' + version + '\n')
  f.write('Requires:\n')
  f.write('Libs: -L' + prefix + '/lib -lgazebo -lgazeboServer\n')
  f.write('Cflags: -I' + prefix + '/include\n')

  
#
# 3rd party packages
#
parseConfigs=['pkg-config --cflags --libs OGRE',
              'xml2-config --cflags --libs', 
      	      'ode-config --cflags --libs',
	            'pkg-config --cflags --libs OIS']

#
# setup the build environment
#
env = Environment (
  CC = 'g++',

  #CCFLAGS = Split ('-pthread -pipe  -W -Wall -O2'),
  CCFLAGS = Split ('-ggdb'),

  CPPPATH = [
   '#.', 
    '#server',
    '#server/models',
    '#libgazebo', 
    '#server/rendering',
    '#server/sensors', 
    '#server/sensors/camera',
    '#server/sensors/ray',
    '#server/physics',
    '#server/physics/ode',
    '#server/controllers',
    '#server/controllers/position2d',
    '#server/controllers/position2d/pioneer2dx',
    ],

  LIBPATH=Split('#libgazebo'),
    
  #LIBS=Split('gazebo boost_python')
  LIBS=Split('gazebo'),

  options=opts
)

Help(opts.GenerateHelpText(env))

install_prefix = env['destdir'] + '/' + env['prefix']

env['BUILDERS']['PkgConfig'] = Builder(action = createPkgConfig)
pkgconfig = env.PkgConfig(target='gazebo.pc', source=Value(env['prefix']))
env.Install(dir=install_prefix+'/lib/pkgconfig', source=pkgconfig)


# DEFAULT list of subdirectories to build
subdirs = ['server','libgazebo', 'player']

#
# Parse all the pacakge configurations
#
if not env.GetOption('clean'):
  for cfg in parseConfigs:
    print "Checking for ["+cfg+"]"
    try:
      env.ParseConfig(cfg)
      print "  Success"
    except OSError,e:
      print "Unable to parse config ["+cfg+"]"
      if cfg.find("OIS") >= 0:
        print "OIS is required, but not found."
        print "  http://sourceforge.net/projects/wgois"
        Exit(1)
      elif cfg.find("OGRE") >= 0:
        print "Ogre3d is required, but not found."
        print "  http://www.ogre3d.org/"
        Exit(1)
      elif cfg.find("ode") >= 0:
        print "ODE is required, but not found."
        print "  http://www.ode.org"
        Exit(1)
  conf = Configure(env, custom_tests = {'CheckODELib' : CheckODELib})
   
  #Check for the ODE library and header
  if not conf.CheckCHeader('ode/ode.h'):
    print "  Error: Install ODE (http://www.ode.org)"
    Exit(1)
   
  # Check for trimesh support in ODE
  if not conf.CheckODELib():
    print '  Error: ODE not compiled with trimesh support.'
    Exit(1)
  env = conf.Finish()
  
  # # Check for boost_python
  #if not conf.CheckLibWithHeader('boost_python', 'boost/python.hpp', 'C'):
  #  print 'Did not find libboost_python exiting'
  #  Exit(1)
  #else:
  #  conf.env.Append(LIBS="boost_python")


staticObjs = []
sharedObjs = []
headers = []

#
# Export the environment
#
Export('env install_prefix version staticObjs sharedObjs headers')

#
# Process subdirectories
#
for subdir in subdirs:
  SConscript('%s/SConscript' % subdir)

#
# Create the gazebo executable
#
gazebo = env.Program('gazebo',staticObjs)

#
# Create static and shared libraries
#
libgazeboServerStatic = env.StaticLibrary('gazeboServer',staticObjs)
libgazeboServerShared = env.SharedLibrary('gazeboServer',sharedObjs)

#
# Install gazebo
#
env.Alias('install', install_prefix)
env.Install(install_prefix+'/bin',gazebo)
env.Install(install_prefix+'/share/gazebo','Media')
env.Install(install_prefix+'/lib',libgazeboServerStatic )
env.Install(install_prefix+'/lib',libgazeboServerShared )
env.Install(install_prefix+'/include/gazebo',headers)
