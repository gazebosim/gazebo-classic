#!/usr/bin/python

import os
import sys
import re

# Get current working directory
cwd = os.getcwd()

# If running from in the tools directory, then add a "/.."
if cwd.endswith("/tools"):
  cwd = cwd + "/.."

# XML always need a results tag, even when there is no errors
if len(sys.argv) > 1 and sys.argv[1] == "xml":
  sys.stderr.write('''<?xml version="1.0" encoding="UTF-8"?>\n''')
  sys.stderr.write('''<results>\n''')
 
# Iterate over all the .proto files
for filename in os.listdir(cwd + "/gazebo/msgs"):
  if filename.endswith('.proto'):
    with open(cwd + "/gazebo/msgs/" + filename, 'r') as f:
      for line in f.readlines():
        # Get the "/// \interface (*)" 
        iface_match = re.search('^///\s\\\\interface\s+.*$', line)
        if iface_match:
          iface = line[iface_match.start()+15:iface_match.end()].strip()

        # Get the "message (*)" 
        msg_match = re.search('^message\s+.*$', line)
        if msg_match:
          msg = line[msg_match.start()+8:msg_match.end()].strip()

      # Make sure the two match
      if iface != msg:
        msg = "Mismatch between \interface and message: %s" % filename
        if len(sys.argv) > 1 and sys.argv[1] == "xml":
          sys.stderr.write('''<error file="%s" line="0" id="0" severity="error" msg="%s">\n''' % (filename, msg))
        else:
          sys.stderr.write(msg + "\n")

if len(sys.argv) > 1 and sys.argv[1] == "xml":
  sys.stderr.write('''</results>\n''')
