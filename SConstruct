version = '0.8'

prefix = ARGUMENTS.get('prefix','/usr/local')

parseConfigs=['pkg-config --cflags --libs OGRE',
              'pkg-config --cflags --libs CEGUI-OGRE',
              'xml2-config --cflags --libs', 
              'pkg-config --cflags --libs CEGUI',
              'pkg-config --cflags --libs playercore']

env = Environment (
  CC = 'g++',

  #CCFLAGS = Split ('-pthread -pipe  -W -Wall -O2'),
  CCFLAGS = Split ('-pthread -pipe -O2'),

  CPPPATH = [
    '#.', 
    '#server',
    '#server/models',
    '#libgazebo', 
    '#server/rendering',
    '#server/sensors', 
    '#server/sensors/camera',
    '#server/physics',
    '#server/physics/ode',
    '/usr/include/python2.4'
    ],

    LIBPATH=Split('#libgazebo'),
    
    LIBS=Split('boost_python python2.4 ode gazebo'),
)

for cfg in parseConfigs:
  env.ParseConfig(cfg)

staticObjs = []
sharedObjs = []

Export('env prefix version staticObjs sharedObjs')

for subdir in ['server', 'libgazebo', 'player']:
  SConscript('%s/SConscript' % subdir)

gazebo = env.Program('gazebo',staticObjs)

libgazeboServerStatic = env.StaticLibrary('gazeboServer',staticObjs)
libgazeboServerShared = env.SharedLibrary('gazeboServer',sharedObjs)

env.Install(prefix+'/bin',gazebo)

env.Alias('install', prefix)
