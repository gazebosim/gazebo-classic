#based on pioneer2d world
#equivalent to the libgazebo/position example

require 'gazeboc'

begin
  client = Gazeboc::Client.new 
  simIface = Gazeboc::SimulationIface.new 
  posIface = Gazeboc::PositionIface.new
rescue Exception => e  
  puts "Gazebo exceptions can be catched on Ruby"
  raise "initialization impossible, bindings installed?"
end

client.Connect 0

posIface.Open client, "pioneer2dx_model1::position_iface_0"
posIface.Lock 1
posIface.data.cmdEnableMotors = 1
posIface.Unlock


while true

  posIface.Lock 1
  posIface.data.cmdVelocity.pos.x = 0.2
  posIface.data.cmdVelocity.yaw = -0.1
  posIface.Unlock
  sleep 1
  pose = posIface.data.pose.pos
  puts "position: (%.3f, %.3f, %.3f)" % [pose.x, pose.y, pose.z]

end

posIface.Close
simIface.Close
