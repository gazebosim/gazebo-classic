import os
import sys
import time 
import commands

exec(open('build.py'))

PKG_CONFIG_VERSION = '0.22'
OGRE_VERSION = '>= 1.6.0'
ODE_VERSION = '>= 0.10.1'


#######
# Create a release
#######
if 'release' in COMMAND_LINE_TARGETS:
  CreateRelease()
  exit()


#######
# Setup the Options
#######
opts = Options()
opts.Add('prefix', 'The install path "prefix"', '/usr/local')
opts.Add('destdir', 'The root directory to install into. Useful mainly for binary package building', '/')
opts.Add('mode','Defines how Gazebo will be built, options available: optimized, profile, debug','debug')


#######
# List of all the required packages
#######
packages = { 
  'OGRE' : {
            'pkgcfg': 'pkg-config --cflags --libs OGRE', 
            'check' : 'OGRE ' + OGRE_VERSION, 
            'msg' : 'Error: Ogre3d and development files are required, but not\
                     found. ',
            'flags' : '',
            'web' : 'http://www.ogre3d.org',
            'required' : True
           },

  'ODE'  : {
            'pkgcfg' : 'pkg-config --cflags --libs ode', 
            'check' : 'ode ' + ODE_VERSION,
            'msg' : 'Error: ODE and development files are required, but not\
                     found.',
            'flags' : '',
            'web' : 'http://www.ode.org',
            'required' : True
           },

  'OpenAL' : {
              'pkgcfg' : 'pkg-config --cflags --libs openal', 
              'check' : 'openal',
              'msg' : 'Warning: OpenAL not found. 3D audio will be disbled',
              'flags' : ['-DHAVE_OPENAL'],
              'web' : 'http://www.openal.org',
              'required' : False
             },

  'XFT' : {
           'pkgcfg' : 'pkg-config --cflags --libs xft',
           'check' : 'xft',
           'msg' : 'Error: ',
           'flags' : '',
           'web' : 'http://www.xft.org',
           'required' : True
          },

  'avformat' : {
                'pkgcfg' : 'pkg-config --cflags --libs libavformat',
                'check' : 'libavformat',
                'msg' : 'Warning: FFMpeg pkg-config not found. MP3 decoding\
                         is disabled',
                'flags' : '',
                'web' : 'http://ffmpeg.mplayerhq.hu/',
                'required' : False
               },

  'avcodec' : {
               'pkgcfg' : 'pkg-config --cflags --libs libavcodec',
               'check' : 'libavcodec',
               'msg' : 'Warning: FFMpeg pkg-config not found. MP3 decoding is\
                         disabled',
               'flags' : '',
               'web' : 'http://ffmpeg.mplayerhq.hu/',
               'required' : False
              },

  'XML2' : {
            'pkgcfg' : 'xml2-config --cflags --libs',
            'check' : '',
            'msg' : 'Error: libxml2 and development files are required, but\
                      not found.',
            'flags' : '',
            'web' : 'http://www.xmlsoft.org',
            'required' : True
           },

  'FLTK' : {
            'pkgcfg' : 'fltk-config --cflags --libs --ldflags --use-gl\
                         --use-images',
            'check' : '',
            'msg' : 'Error: libfltk & development files are required, but not\
                     found.',
            'flags' : '',
            'web' : 'http://www.fltk.org',
            'required' : True
           },
  'Player' : {
              'pkgcfg' : 'pkg-config --cflags --libs playerc++',
              'check' : 'playerc++',
              'msg' : 'Warning: Player not found, bindings will not be built.',
              'flags' : ['-DHAVE_PLAYER'],
              'web' :  'http://playerstage.sourceforge.net',
              'required' : False
           },
  'event' : {
             'header' : 'event.h',
             'lib' : 'event',
             'msg' : 'Warning: libevent not found. Webgazebo will no be built.',
             'flags' : ['-DHAVE_EVENT'],
             'web' :  'http://www.monkey.org/~provos/libevent/',
            'required' : False
           },
  'yaml' : {
             'header' : 'yaml.h',
             'lib' : 'yaml',
             'msg' : 'Warning: yaml not found. Webgazebo will not be built.',
             'flags' : ['-DHAVE_YAML'],
             'web' :  'http://www.yaml.org',
             'required' : False
           },
  'websim' : {
              'pkgcfg' : 'pkg-config --cflags --libs websim',
              'check' : 'websim',
              'msg' : 'Warning: websim not found. Webgazebo will not be built.',
              'flags' : ['-DHAVE_WEBSIM'],
              'web' :  'http://www.playerstage.sourceforge.net',
              'required' : False
           },
  'glib' : {
              'pkgcfg' : 'pkg-config --cflags --libs glib-2.0',
              'check' : 'glib-2.0',
              'msg' : 'Warning: glib not found. Webgazebo will not be built.',
              'flags' : '',
              'web' :  'http://www.gtk.org',
              'required' : False
           },
} 

#######
# setup the build environment
#######
env = Environment (
  CC = 'gcc',
  CXX = 'g++',
  CPPPATH = [ '#.' ],
  LIBPATH=Split('#libgazebo #server/audio_video'),
    
  #LIBS=Split('gazebo boost_python')
  LIBS=Split('gazebo boost_signals'),
  LINKFLAGS=Split('-export-dynamic'),

  TARFLAGS = '-c -z',
  TARSUFFIX = '.tar.gz',

  options = opts,

  # Static object compile string
  CXXCOMSTR = 'Compiling $TARGET',
  CCCOMSTR = 'Compiling $TARGET',
  LINKCOMSTR = 'Linking $TARGET',
  ARCOMSTR = 'Linking $TARGET',

  # Shared object compile strings
  SHCXXCOMSTR = 'Compiling $TARGET',
  SHCCCOMSTR = 'Compiling $TARGET',
  SHLINKCOMSTR = 'Linking $TARGET',
  SHARCOMSTR = 'Linking $TARGET',

  HAS_ERROR = False,
)


#######
# DEFAULT list of subdirectories to build
#######
subdirs = ['libgazebo','server', 'player']
# Need test for <event.h>
try:
  env["CCFLAGS"].index("-DHAVE_WEBSIM")
  env["CCFLAGS"].index("-DHAVE_YAML")
  env["CCFLAGS"].index("-DHAVE_EVENT")
  subdirs.append('webgazebo')
except ValueError, e:
  garbage = 1

#######
# Generate help text
#######
Help(opts.GenerateHelpText(env))


#######
# Setup default install destination
#######
if env['destdir'] != '/':
  install_prefix = env['destdir'] + '/' + env['prefix']
else:
  install_prefix = env['prefix']


#######
# Create the custom builder
#######
env['BUILDERS']['PkgConfig'] = Builder(action = createPkgConfig)
pkgconfig = env.PkgConfig(target='libgazeboServer.pc', source=Value(install_prefix))
env.Install(dir=install_prefix+'/lib/pkgconfig', source=pkgconfig)

env['BUILDERS']['RCConfig'] = Builder(action = createGazeborc)
rcconfig = env.RCConfig(target='gazeborc', source=Value(install_prefix))

#######
# Set the compile mode
#######
if env['mode'] == 'debug':
  env['CCFLAGS'] += Split('-ggdb -g2 -Wall -Wno-deprecated -fPIC')
elif env['mode'] == 'profile':
  env['CCFLAGS'] += Split('-ggdb -g2 -pg -fPIC') 
  env['LINKFLAGS'] += Split('-pg') 
elif env['mode'] == 'optimized':
  env['CCFLAGS'] += Split('-O3 -fPIC') 

optimize_for_cpu(env)

#######
# Conifgure the system
#######
Config(env, packages)


if env['HAS_ERROR'] == True:
  print "Build failed due to missing packages. See above errors"
  Exit()

#######
# Export the environment
#######
#staticObjs = []
sharedObjs = []
headers = []
Export('env install_prefix version sharedObjs headers subdirs')


#######
# Parse subdirectories
#######
for subdir in subdirs:
  print 'Parsing sub directory ' + subdir
  SConscript('%s/SConscript' % subdir)

print ''


#######
# Create the gazebo executable and libraries
#######
if not 'configure' in COMMAND_LINE_TARGETS:
  #Progress('Evaluating $TARGET\n')

  gazebo = env.Program('gazebo',sharedObjs)
  Depends(gazebo, 'libgazebo/libgazebo.so')

  #libgazeboServerStatic = env.StaticLibrary('gazeboServer', staticObjs)
  libgazeboServerShared = env.SharedLibrary('gazeboServer', sharedObjs)

  #
  # Install gazebo
  #
  env.Alias('install', install_prefix)
  env.Install(install_prefix+'/bin',gazebo)
  env.Install(install_prefix+'/include/gazebo',headers)
  #env.Install(install_prefix+'/lib',libgazeboServerStatic )
  env.Install(install_prefix+'/lib',libgazeboServerShared )
  env.Install(install_prefix+'/share/gazebo','worlds')
  env.Install(install_prefix+'/share/gazebo','Media')
else:
  print 'Configure done'
  Exit()

