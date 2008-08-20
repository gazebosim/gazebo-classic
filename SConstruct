import os
import sys
import time 
import commands

exec(open('build.py'))

if 'release' in COMMAND_LINE_TARGETS:
  CreateRelease()
  exit()

#
# Setup the Options
#
opts = Options()
opts.Add('prefix', 'The install path "prefix"', '/usr/local')
opts.Add('destdir', 'The root directory to install into. Useful mainly for binary package building', '/')
opts.Add('mode','Defines how Gazebo will be built, options available: optimized, profile, debug','debug')
opts.Add('with_audio','Builds support for 3d sound. Options available: no, yes ', 'no')

#
# 3rd party packages
#
parseConfigs=['pkg-config --cflags --libs OGRE',
              'xml2-config --cflags --libs', 
      	      'ode-config --cflags --libs',
              'fltk-config --cflags --libs --ldflags --use-gl --use-images',
              'pkg-config --cflags --libs xft'
              ]

#
# setup the build environment
#
env = Environment (
  CC = 'g++',

  CPPPATH = [
   '#.', 
   '#server',
   '#server/models',
   '#server/gui',
   '#server/gui/fltk',
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
  LIBS=Split('gazebo boost_signals'),
  LINKFLAGS=Split('-export-dynamic'),

  TARFLAGS = '-c -z',
  TARSUFFIX = '.tar.gz',

  options=opts
)

Help(opts.GenerateHelpText(env))

if env['destdir'] != '/':
  install_prefix = env['destdir'] + '/' + env['prefix']
else:
  install_prefix = env['prefix']


env['BUILDERS']['PkgConfig'] = Builder(action = createPkgConfig)
pkgconfig = env.PkgConfig(target='libgazeboServer.pc', source=Value(install_prefix))
env.Install(dir=install_prefix+'/lib/pkgconfig', source=pkgconfig)

env['BUILDERS']['RCConfig'] = Builder(action = createGazeborc)
rcconfig = env.RCConfig(target='gazeborc', source=Value(install_prefix))

# DEFAULT list of subdirectories to build
subdirs = ['libgazebo','server', 'player']

# Set the compile mode
if env['mode'] == 'debug':
  env['CCFLAGS'] += Split('-ggdb -g2 -Wall -Wno-deprecated')
elif env['mode'] == 'profile':
  env['CCFLAGS'] += Split('-ggdb -g2 -pg') 
  env['LINKFLAGS'] += Split('-pg') 
elif env['mode'] == 'optimized':
  env['CCFLAGS'] += Split('-O3') 

if env['with_audio'] == 'yes':
  parseConfigs+=['pkg-config --cflags --libs OgreAL']


optimize_for_cpu(env);

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
      if cfg.find("OGRE") >= 0:
        print "Ogre3d and development files are required, but not found."
        print "  http://www.ogre3d.org/"
        Exit(1)
      elif cfg.find("ode") >= 0:
        print "ODE and development files are required, but not found."
        print "  http://www.ode.org"
        Exit(1)
      elif cfg.find("xml2") >= 0:
        print "libxml2 and development files are required, but not found."
        print "  http://www.xmlsoft.org"
        Exit(1)
      elif cfg.find("fltk") >= 0:
        print "libfltk & development files are required, but not found."
        print "  http://www.fltk.org"
        Exit(1)
      elif cfg.find("OgreAL") >= 0:
        print "You requested to activate the support for 3D audio. But OgreAL is not found."
        print "http://www.ogre3d.org/phpBB2addons/viewtopic.php?t=3293"
        Exit(1)

   #FIXME: if this check fails, it makes it fail the check for ODE  
  # This test should be done outside of the configure context below, because
  # otherwise it tries to link against the not-yet-built libgazebo, causing
  # the test to always fail.
  simpleenv = Environment()
  simpleconf = Configure(env)
  if not simpleconf.CheckLibWithHeader('ltdl','ltdl.h','CXX'):
    print "  Warning: Failed to find ltdl, no plugin support will be included"
    env["HAVE_LTDL"]=False
  else:
    env["HAVE_LTDL"]=True
    env["CCFLAGS"].append("-DHAVE_LTDL")
  simpleconf.Finish()

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
Export('env install_prefix version staticObjs sharedObjs headers subdirs')

#
# Process subdirectories
#
for subdir in subdirs:
  print subdir
  SConscript('%s/SConscript' % subdir)

#
# Create the gazebo executable
#
gazebo = env.Program('gazebo',staticObjs)

Depends(gazebo, 'libgazebo/libgazebo.so')
#
# Create static and shared libraries for the server
#
libgazeboServerStatic = env.StaticLibrary('gazeboServer', staticObjs)
libgazeboServerShared = env.SharedLibrary('gazeboServer', sharedObjs)

#
# Install gazebo
#
env.Alias('install', install_prefix)
env.Install(install_prefix+'/bin',gazebo)
env.Install(install_prefix+'/share/gazebo','Media')
env.Install(install_prefix+'/include/gazebo',headers)
env.Install(install_prefix+'/lib',libgazeboServerStatic )
env.Install(install_prefix+'/lib',libgazeboServerShared )
env.Install(install_prefix+'/share/gazebo','worlds')
