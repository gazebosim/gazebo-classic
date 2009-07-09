#TODO:test this
require 'gazeboc'

begin
  client = Gazeboc::Client.new 
  bumperIface = Gazeboc::BumperIface.new
rescue Exception => e  
  puts "Gazebo exceptions can be catched on Ruby"
  raise "initialization impossible, bindings installed?"
end

client.Connect 0

posIface.Open client, "PUT_BUMPER_IFACE_HERE"
posIface.Lock 1
posIface.data.cmdEnableMotors = 1
posIface.Unlock

10.times do

  bumIface.Lock 1
  count = bumperIface.data.bumper_count
  
  count.times do |i]
    puts bumberIface.data.bumpers[i].to_s
  end
  bumIface.Unlock
  sleep(0.1
end

bumperIface.Close
