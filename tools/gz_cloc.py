#!/usr/bin/python

import subprocess
import datetime

# Warning: This script will update the current repository!!

# How to use
# 1. This run this script in the root director of Gazebo.
# 2. The output will be written to /tmp/gz_loc.csv
# 3. Load into a Google spreadsheet.
# 4. Generate a graph, and select Time Line as the graph type.

start_year = 2007

f = open('/tmp/gz_loc.csv', 'w')

now = datetime.datetime.now()

f.write("date,blank,comment,code\n")

for y in range(start_year, now.year+1):
  if y == now.year:
    end_month = now.month + 1
  else:
    end_month = 13

  for m in range(1,end_month):
    date = str(y) + "-" + str(m) 
    cmd = "hg up -d \"" + date + "\""
    print cmd
    os.system(cmd)

    cmd = "cloc --force-lang=\"C++\",cc --force-lang=\"C++\",c --force-lang=\"C++\",hh --force-lang=\"C++\",h --force-lang=\"C++\",hpp --exclude-lang=\"XML\",\"HTML\",\"XSD\",\"Python\",\"make\",\"Bourne Shell\",\"Javascript\",\"CSS\",\"m4\",\"Ruby\",\"DOS Batch\" --exclude_dir=deps,Media,media,cmake,doc,build --csv --quiet --progress-rate=0 * | tail -n 1"
    proc = subprocess.Popen([cmd], stdout=subprocess.PIPE, shell=True)
    (out, err) = proc.communicate()
    out_parts = out.split(',')
    f.write("%d/01/%d, %s, %s, %s" % (m, y, out_parts[2], out_parts[3], out_parts[4]))

f.close()
