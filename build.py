import sys
import SCons.Util
import SCons.SConf as SConf
import string
import os
import re

version = '0.8-pre2'

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

