import os
import sys
import time

version = '0.8'

# TESTING:
#odeTestBuild = Builder(action="echo g++ -o $TARGET $_CPPINCFLAGS $SOURCE $_LIBDIRFLAGS $_LIBFLAGS >> /tmp/natedogg")

# 3rd party packages
parseConfigs=['pkg-config --cflags --libs OGRE',
              'xml2-config --cflags --libs', 
              'pkg-config --cflags --libs playerc++',
              #'pkg-config --cflags --libs playerxdr',
      	      'ode-config --cflags --libs',
	            'pkg-config --cflags --libs OIS']
	            #'python-config --cflags --libs']

opts = Options()
#opts.Add('PREFIX', 'The install path "prefix"', '/usr/local')
opts.Add('DESTDIR', 'The root directory to install into. Useful mainly for binary package building', '/')

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

prefix = ARGUMENTS.get('prefix','/usr/local')
install_prefix = env['DESTDIR'] + '/' + prefix
# TESTING:
#env.Append(BUILDERS = {'odeTestBuild' : odeTestBuild})

# DEFAULT list of subdirectories to build
subdirs = ['server','libgazebo', 'player']

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
      elif cfg.find("player") >=0:
        print "\n================================================================"
        print "Player not found, bindings will not be built."
        print "  To install player visit(http://playerstage.sourceforge.net)"
        subdirs.remove('player')
        print "================================================================"
        #time.sleep(3)
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

#
# Export the environment
#
Export('env prefix version staticObjs sharedObjs')

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
