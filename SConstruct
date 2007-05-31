version = '0.8'

prefix = ARGUMENTS.get('prefix','/usr/local')


# 3rd party packages
parseConfigs=['pkg-config --cflags --libs OGRE',
              'pkg-config --cflags --libs CEGUI-OGRE',
              'xml2-config --cflags --libs', 
              'pkg-config --cflags --libs CEGUI',
              'pkg-config --cflags --libs playercore',
              'pkg-config --cflags --libs playerxdr',
	      'ode-config --cflags --libs',
	      'pkg-config --cflags --libs OIS',
	      'python-config']

env = Environment (
  CC = 'g++',

  CCFLAGS = Split ('-pthread -pipe  -W -Wall -O2'),
  #CCFLAGS = Split ('-pthread -pipe'),

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
    '/usr/include/python2.4',
    ],

    LIBPATH=Split('#libgazebo'),
    
    LIBS=Split('gazebo boost_python')
)


# Parse all the pacakge configurations
for cfg in parseConfigs:
  try:
    env.ParseConfig(cfg)
  except OSError:
    print "Unable to parse config ["+cfg+"]"
    if cfg.find("OIS") >= 0:
      print "Install Open Input System (http://sourceforge.net/projects/wgois)\n"
    elif cfg.find("OGRE") >= 0:
      print "Install Ogre3d (http://www.ogre3d.org/)\n"
    elif cfg.find("player") >=0:
      print "Install Player (http://playerstage.sourceforge.net)\n"
    elif cfg.find("CEGUI") >=0:
      print "Install CEGUI (http://www.cegui.org.uk/wiki/index.php/Main_Page)\n"
    Exit(1)

conf = Configure(env)

# Check for the necessary headers
#if not conf.CheckHeader('boost/python.hpp'):
#  print 'Did not find boost/python.hpp exiting'
#  Exit(1)

env = conf.Finish()

  

staticObjs = []
sharedObjs = []

Export('env prefix version staticObjs sharedObjs')

for subdir in ['server', 'libgazebo', 'player']:
  SConscript('%s/SConscript' % subdir)

gazebo = env.Program('gazebo',staticObjs)

libgazeboServerStatic = env.StaticLibrary('gazeboServer',staticObjs)
libgazeboServerShared = env.SharedLibrary('gazeboServer',sharedObjs)

env.Install(prefix+'/bin',gazebo)
env.Install(prefix+'/share/gazebo','Media')

env.Alias('install', prefix)
