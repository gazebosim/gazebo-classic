#!/usr/bin/python

import os
import sys
import subprocess
import re
import datetime

process = subprocess.Popen([sys.argv[1], '-h'], stderr=subprocess.PIPE)
stdout, stderr = process.communicate()
help_txt = stderr

lines = help_txt.splitlines()

if len(sys.argv) > 2:
  f = open(sys.argv[2], 'w')
else:
  f = sys.stdout

i = 0
for i in range(len(lines)):
  if i == 0:
    f.write("%s\n" % lines[i])
    f.write("=============================================\n\n")
  elif i == 2:
    f.write("## SYNOPSIS\n\n")
    f.write('%s\n\n' % lines[i])
  elif i == 4:
    f.write("## DESCRIPTION\n\n")
    f.write("%s\n\n" % lines[i])
  elif lines[i] == "See also:":
    f.write("## SEE ALSO\n")
    i = i + 1

    # It's assumed that "See also:" is the last set of help text
    while i < len(lines):
      f.write("%s\n" % lines[i])
      i = i + 1
  else:
    if lines[i] == "Allowed options:" or lines[i] == "Options:":
      i = i + 1
      f.write("## OPTIONS\n\n")
      while i < len(lines) and len(lines[i]) > 0 and lines[i][0] == " ":
        line = lines[i].replace(" [ ", ", ")
        line = line.replace(' ]', '')
        line = line.strip()
        line = re.sub('\s+', ' ', line)
        line = re.sub('\s([A-Z].*)', r' :\n \1', line)
        i = i + 1
        while i < len(lines) and len(lines[i]) > 0 and lines[i][0] == " " and re.match(r'^   ', lines[i]):
          line += " " + lines[i].strip()
          i = i + 1
        f.write("* %s\n"%line)
      f.write("\n")

    if lines[i] == "Commands:":
      i = i + 1
      f.write("## COMMAND ELEMENTS\n\n")
      while i < len(lines) and len(lines[i]) > 0 and lines[i][0] == " ":
        line = lines[i].strip()
        line = re.sub('\s+', ' ', line)
        line = re.sub('\s([A-Z].*\.$)', r'**:\n\1', line)
        f.write("* **%s\n" % line)
        i = i + 1
      f.write("\n")

f.write("""
## AUTHOR
  Open Source Robotics Foundation

## COPYRIGHT 
  Copyright (C) 2012-%s Open Source Robotics Foundation
       
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
""" % (datetime.datetime.now().year))
