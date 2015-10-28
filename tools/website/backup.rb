#!/usr/bin/env ruby

# Copyright (C) 2012-2015 Open Source Robotics Foundation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# This script removes old snapshots of the gazebosim.org server, and creates
# a new snapshot if the most recent snapshot is older than 1 day. Snapshots
# are removed if they are older than one week.
#
# A cronjob runs on gazebosim.org that assumes this file is located in /home/ubuntu/bin/

require 'rubygems'
require 'net/http'
require 'net/ssh'
require 'aws-sdk'

if ARGV.size < 2
  puts "backup.rb <access_key> <secret_key>"
  abort("")
end

# Two weeks in seconds
two_weeks = 2*7*24*60*60

# One day in seconds
one_day = 24*60*60

# Timestamp of the last snapshot
last_snapshot = 0

AWS.config(
  :access_key_id => ARGV[0],
  :secret_access_key => ARGV[1])

ec2 = AWS::EC2.new

# Find old gazebo sim snapshots
ec2.snapshots.filter("tag:Name","gazebosim.org").each {|snapshot|

  # Only consider snapshot with Name == gazebosim.org
  if snapshot.tags["Name"] == "gazebosim.org"

    # Only delete a snap shot if it's older than two weeks.
    if Time.now.to_i - snapshot.tags["Time"].to_i > two_weeks
      puts "Deleting snapshot with id #{snapshot.id}"
      snapshot.delete
    else
      # Store the timestamp of the most recent snapshot
      last_snapshot = [last_snapshot, snapshot.tags["Time"].to_i].max
    end
  end
}

# Create a new snapshot if the most recent snapshot is older than one day.
if Time.now.to_i - last_snapshot >= one_day
  # Find the gazebosim volume
  ec2.volumes.each {|volume|
    # Only consider volumes that are in use
    if volume.status == :in_use
      volume.attachments.each {|attachment| 

        # Check for the gazebosim.org name
        if attachment.status == :attached &&
          attachment.instance.tags["Name"] == "gazebosim.org"
  
          # Create a snapshot
          puts "Creating snapshot of gazebosim volume id #{volume.id}"
          snapshot = volume.create_snapshot("gazebosim_#{Time.now.to_i}")
          snapshot.tags.Name = "gazebosim.org"
          snapshot.add_tag("Time", :value => "#{Time.now.to_i}")
        end
      }
    end
  }
end
