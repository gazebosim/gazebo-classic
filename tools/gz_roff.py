#!/usr/bin/python

import os
import sys
import subprocess
import re
import datetime

f = sys.stdout

def outputCommand(_command):
  process = subprocess.Popen(['gz', _command, '-h'], stderr=subprocess.PIPE)
  stdout, stderr = process.communicate()
  help_txt = stderr
  
  lines = help_txt.splitlines()

  i = 0
  
  f.write(".UNINDENT\n")
  f.write(".SS %s\n" % _command)
  f.write(".sp\n.nf\n.ft C\n")
  f.write("%s\n" % lines[i].strip())
  f.write(".ft P\n.fi\n.sp\n")
  
  i = i + 1
  
  while lines[i] != "Options:":
    f.write("%s\n" % lines[i].strip())
    i = i + 1
  
  i = i + 1
  
  f.write(".sp\nOptions:\n")
  f.write(".INDENT 0.0\n")
  
  while i < len(lines):
    line = lines[i]
    line = re.sub('\s+', ' ', line).strip()
    line = line.replace(" [",",")
    line = line.replace(" ]","")
    line = re.sub('\s([A-Z].*)', r'\n.\n\1', line)

    line = line.replace(" arg\n","\\fR=\\fIarg\\fR\n")
    line = line.replace("-","\\-")
  
    while i+1 < len(lines) and len(lines[i+1]) > 0 and lines[i+1][0] == " " and re.match(r'^   ', lines[i+1]):
      i = i + 1
      line += " " + lines[i].strip()
 
    if len(line) > 0:
      f.write(".TP\n")
      f.write('.B %s\n' % line)
  
    i = i + 1


f.write('.TH "GZ" "1" "March 2014" "" ""\n')
f.write('.\n')
f.write('.SH "NAME"\n')
f.write('\\fBgz\\fR \- Gazebo command line tool for control and analysis.\n')
f.write('.\n')
f.write('.SH "SYNPOSIS"\n')
f.write('\\fBgz\\fR \\fIcommand\\fR [option]\.\.\. [argument]\.\.\.\n')
f.write('.\n')
f.write('.UNINDENT\n')
f.write('.SH COMMANDS\n')

process = subprocess.Popen(['gz', 'debug'], stdout=subprocess.PIPE)
stdout, stderr = process.communicate()
helpTxt = stdout
  
commands = helpTxt.splitlines()

for cmd in commands:
  outputCommand(cmd)

f.close()
