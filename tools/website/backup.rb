#!/usr/bin/env ruby
# Description:
# This script removes old snapshots of the gazebosim.org server, and creates
# a new snapshot if the most recent snapshot is older than 1 day. Snapshots
# are removed if they are older than one week.

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
