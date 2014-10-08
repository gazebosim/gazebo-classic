#!/usr/bin/env ruby

# Warning: This script will update the current repository!!
#
# How to use
# 1. Run this script in the root directory of Gazebo.
# 2. The output will be written to /tmp/gz_loc.csv
# 3. Load into a Google spreadsheet.
# 4. Generate a graph, and select Time Line as the graph type.

printf("Warning: This will modify the current repository.\n")
printf("Do you want to continue (Y/n): ")
char = gets

if char != "Y\n"
  abort
end

startYear = 2007

f = File.open('/tmp/gz_loc.csv', 'w')

time = Time.new

f.printf("date,blank,comment,code\n")

# Loop through all the years.
for y in (startYear..time.year)

  # Get the end month
  endMonth = y == time.year ? time.month : 12

  # Loop through all the months.
  for m in (1..endMonth)
    date = y.to_s + "-" + m.to_s

    # Update the repo
    `hg up -C -d "#{date}"`

    # Count lines of code
    result = `cloc --force-lang=\"C++\",cc --force-lang=\"C++\",c --force-lang=\"C++\",hh --force-lang=\"C++\",h --force-lang=\"C++\",hpp --exclude_dir=deps,Media,media,cmake,doc,build --csv --quiet --progress-rate=0 *`

    lines = result.split("\n")

    # Grab just C++ results
    lines.each do |line|
      if line.include?("C++")
        parts = line.split(",")
        printf("%d/01/%d, %s, %s, %s\n", m, y, parts[2], parts[3], parts[4])
        f.printf("%d/01/%d, %s, %s, %s\n", m, y, parts[2], parts[3], parts[4])
      end
    end
  end
end

f.close()
