import sys
import SCons.Util
import SCons.SConf as SConf
import string
import os
import re

version = '0.8-pre3'

#
# Function used to test for ODE library, and TriMesh support
#
ode_test_source_file = """
#include <ode/ode.h>
int main()
{
  dGeomTriMeshDataCreate();
  return 0;
}
"""

def CreateRelease():
  os.system('scons -c')
  sysCmd = 'mkdir gazebo-'+version
  os.system(sysCmd)
  sysCmd = "cp -r server Media doc examples libgazebo player worlds SConstruct AUTHORS ChangeLog NEWS README TODO build.py gazebo-"+version
  os.system(sysCmd)
  sysCmd = "find gazebo-"+version+"/. -name .svn -exec rm -rf {} \; 2>>/dev/null"
  os.system(sysCmd)
  print 'Creating tarball gazebo-'+version+'.tar.gz'
  sysCmd = 'tar czf gazebo-'+version+'.tar.gz gazebo-'+version
  os.system(sysCmd)
  sysCmd = "rm -rf gazebo-"+version
  os.system(sysCmd)

  
def CheckPkgConfig(context, version):
  context.Message( 'Checking for pkg-config... ' )
  ret = context.TryAction('pkg-config --atleast-pkgconfig-version=%s' % version)[0]
  context.Result( ret )
  return ret

def CheckPkg(context, name):
  context.Message( 'Checking for %s...' % name)
  pkg_config_cmd = "pkg-config"
  if os.environ.has_key("PKG_CONFIG_PATH"):
    pkg_config_cmd = "PKG_CONFIG_PATH="+os.environ["PKG_CONFIG_PATH"]+" pkg-config"  
  action = '%s --exists \'%s\'' % (pkg_config_cmd, name);

  ret = context.TryAction(action)[0]
  context.Result(ret)
  return ret

def CheckODELib(context):
  context.Message('Checking for ODE trimesh support...')
  oldLibs = context.env['LIBS']
  context.env.Replace(LIBS='ode')
  result = context.TryLink(ode_test_source_file, '.cpp')
  context.Result(result)
  context.env.Replace(LIBS=oldLibs)
  return result

#
# Create the pkg-config file
#
def createPkgConfig(target,source, env):
  f = open(str(target[0]), 'wb')
  prefix = source[0].get_contents()
  f.write('prefix=' + prefix + '\n')
  f.write('Name: gazebo\n')
  f.write('Description: Simplified interface to Player\n')
  f.write('Version:' + version + '\n')
  f.write('Requires: OGRE\n')
  f.write('Libs: -L' + prefix + '/lib -lgazeboServer -lode\n')
  f.write('Cflags: -I' + prefix + '/include\n')
  f.close()

#
# Create the .gazeborc file
#
def createGazeborc(target, source, env):
  f = open(str(target[0]),'wb')
  prefix=source[0].get_contents()
  ogre=commands.getoutput('pkg-config --variable=plugindir OGRE')
  f.write('<?xml version="1.0"?>\n')
  f.write('<gazeborc>\n')
  f.write('  <gazeboPath>' + prefix + '/share/gazebo</gazeboPath>\n')
  f.write('  <ogrePath>' + ogre + '</ogrePath>\n')
  f.write('  <RTTMode>PBuffer</RTTMode>\n')
  f.write('</gazeborc>\n')
  f.close()
  # Use a python command because scons won't copy files to a home directory
  if not os.path.exists(os.environ['HOME']+'/.gazeborc'):
    os.system('cp gazeborc ~/.gazeborc')




#
# Add optimizations to compiler flags based on the architecture
#
#TODO:search architecture
def optimize_for_cpu(env):
  cpuAtr = ['mmx', 'sse', 'sse2', 'sse3', '3dnow']
  line_atr =""
  if (os.path.isfile("/proc/cpuinfo")):
    f_proc = open("/proc/cpuinfo")
    for line in f_proc:
      if (re.match('flags', line)): 
        for atr in cpuAtr:
          if (re.search(atr, line) and not re.search(atr, line_atr)):
            line_atr+= "-m" + atr + " "
            
  env['CCFLAGS'] += Split (line_atr)
  print "detected CPU atributes: " +line_atr


def Config(env, packages):
  simpleEnv = Environment(CC="g++")

  # Setup the configuration environment.
  conf = Configure(simpleEnv, custom_tests = {'CheckODELib' : CheckODELib,
                                        'CheckPkgConfig' : CheckPkgConfig,
                                        'CheckPkg' : CheckPkg})
  
  print '\n=== 3rd Party Dependency Checks === '
  
  # Check for pkg-config 
  if not conf.CheckPkgConfig(PKG_CONFIG_VERSION):
    print '  Error: pkg-config version >= ' + PKG_CONFIG_VERSION + ' not found.'
    Exit(1)
  
  #
  # Parse all the pacakge configurations
  #
  for key in packages:
    pkgcfg =''
    header=''
    lib=''
    check=''
    requred = True

    if packages[key].has_key('pkgcfg'):
      pkgcfg = packages[key]['pkgcfg']

    if packages[key].has_key('header'):
      header = packages[key]['header']

    if packages[key].has_key('lib'):
      lib = packages[key]['lib']

    if packages[key].has_key('check'):
      check = packages[key]['check']

    if packages[key].has_key('required'):
      required = packages[key]['required']


    msg = packages[key]['msg']
    web = packages[key]['web']
    flags = packages[key]['flags']

    docfg = True
    valid = True
  
    # Check for the package
    if check and not conf.CheckPkg(check):
      docfg = False
      valid = False
      print '  !!' + msg + key + ' not found.'
      print '  See: ' + web
      if required:
        env['HAS_ERROR'] = True;
 
    # Try parsing the pkg-config
    if docfg and pkgcfg:
      try:
        if not check:
          print "Checking for "+key+"...",
        pkgcfg = "PKG_CONFIG_PATH="+os.environ["PKG_CONFIG_PATH"]+" "+pkgcfg;
        env.ParseConfig(pkgcfg)
        if not check:
          print 'yes'
      except OSError,e:
        valid = False
        if not check:
          print 'no'
        print "Unable to parse config ["+pkgcfg+"]"
        print '  !!' + msg + key + ' not found.'
        print '  See: ' + web
        if required:
          env['HAS_ERROR'] = True;
    elif header and lib:
      if not conf.CheckLibWithHeader(lib, header, 'c'):
        valid = False
        print msg
      else:
        env.Append(LIBS = lib)
        valid = True
  
    # If valid so far, apply any flags to the environment
    if valid:
        env.Append(CCFLAGS = flags)
  
  # Check for trimesh support in ODE
  #if not conf.CheckODELib():
  #  print '  Error: ODE not compiled with trimesh support.'
  #  Exit(1)

  if not conf.CheckCHeader('avformat.h'):
    if conf.CheckCHeader('libavformat/avformat.h'):
      env.Append( CCFLAGS = '-DSPECIAL_LIBAVFORMAT')
    else:
      print "Unable to find libavformat"

  if not conf.CheckCHeader('avcodec.h'):
    if conf.CheckCHeader('libavcodec/avcodec.h'):
      env.Append( CCFLAGS = '-DSPECIAL_LIBAVCODEC')
    else:
      print "Unable to find libavcodec"

    
  simpleEnv = conf.Finish()
  
  #simpleenv = Environment(CPPPATH="/usr/include/AL")
  #simpleconf = Configure(simpleenv)
  #
  ## Test for libtool
  #if not simpleconf.CheckLibWithHeader('ltdl','ltdl.h','CXX'):
  #  print "  Warning: Failed to find ltdl, no plugin support will be included"
  #  env["HAVE_LTDL"]=False
  #else:
  #  env["HAVE_LTDL"]=True
  #  env.Append(CCFLAGS = " -DHAVE_LTDL")
  #  env.Append(LIBS = "ltdl")
 
  #if not simpleconf.CheckHeader('boost/signal.hpp',language='C++'):
  #  print "Error: Boost signals not found. Please install."
  #  Exit(0)
  #
  #simpleconf.Finish()
  
  # # Check for boost_python
  #if not conf.CheckLibWithHeader('boost_python', 'boost/python.hpp', 'C'):
  #  print 'Did not find libboost_python exiting'
  #  Exit(1)
  #else:
  #  conf.env.Append(LIBS="boost_python")
  
  print '=== Done ===\n'


