#!/usr/bin/python
import os

for f in os.listdir('.'):
  cmd = "sed -i -e 's/\.\.\/include\///g' " + f
  print cmd
  os.system(cmd)

#sed -i -e s/\.\.\/include\///g
