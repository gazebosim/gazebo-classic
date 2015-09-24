/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/ArduCopterPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduCopterPlugin)

////////////////////////////////////////////////////////////////////////////////
ArduCopterPlugin::ArduCopterPlugin()
{
  // socket
  this->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
  fcntl(this->handle, F_SETFD, FD_CLOEXEC);
  int one = 1;
  setsockopt(this->handle, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  this->bind("127.0.0.1", 9002);

  setsockopt(this->handle, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  //fcntl(this->handle, F_SETFL, fcntl(this->handle, F_GETFL, 0) & ~O_NONBLOCK);
  fcntl(this->handle, F_SETFL, fcntl(this->handle, F_GETFL, 0) | O_NONBLOCK);
}

/////////////////////////////////////////////////
ArduCopterPlugin::~ArduCopterPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void ArduCopterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ArduCopterPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "ArduCopterPlugin _sdf pointer is NULL");

  this->model = _model;

  // per rotor
  if (_sdf->HasElement("rotor"))
  {
    sdf::ElementPtr rotorSDF = _sdf->GetElement("rotor");

    while (rotorSDF)
    {
      Rotor rotor;

      if (rotorSDF->HasAttribute("id"))
      {
        rotor.id = rotorSDF->GetAttribute("id")->Get(rotor.id);
      }
      else
      {
        gzwarn << "[ArduCopterPlugin] id attribute not specified,"
               << " check motorNumber.\n";
        if (rotorSDF->HasElement("motorNumber"))
        {
          rotor.id = rotorSDF->Get<int>("motorNumber");
        }
        else
        {
          gzwarn << "[ArduCopterPlugin] motorNumber not specified,"
                 << " use order parsed.\n";
          rotor.id = this->rotors.size();
        }
      }

      if (rotorSDF->HasElement("jointName"))
        rotor.jointName = rotorSDF->Get<std::string>("jointName");
      else
        gzerr << "[ArduCopterPlugin] Please specify a jointName,"
              << " where the rotor is attached.\n";
      // Get the pointer to the joint.
      rotor.joint = _model->GetJoint(rotor.jointName);
      if (rotor.joint == NULL)
        gzthrow("[ArduCopterPlugin] Couldn't find specified"
              << " joint [" << rotor.jointName << "].");

      if (rotorSDF->HasElement("linkName"))
        rotor.linkName = rotorSDF->Get<std::string>("linkName");
      else
        gzerr << "[ArduCopterPlugin] Please specify a linkName for"
              << " the rotor\n";
      rotor.link = _model->GetLink(rotor.linkName);
      if (rotor.link == NULL)
        gzthrow("[ArduCopterPlugin] Couldn't find specified"
              << " link [" << rotor.linkName << "].");

      if (rotorSDF->HasElement("turningDirection"))
      {
        std::string turningDirection = rotorSDF->Get<std::string>(
          "turningDirection");
        // special cases mimic from rotors_gazebo_plugins
        if (turningDirection == "cw")
          rotor.multiplier = -1;
        else if (turningDirection == "ccw")
          rotor.multiplier = 1;
        else
        {
          gzdbg << "[ArduCopterPlugin] not string,"
                << " check turningDirection as float\n";
          rotor.multiplier = rotorSDF->Get<double>("turningDirection");
        }
      }
      else
      {
        rotor.multiplier = 1;
        gzerr << "[ArduCopterPlugin] Please specify a turning"
              << " direction multiplier ('cw' or 'ccw'). Default 1.\n";
      }

      getSdfParam<double>(rotorSDF, "rotorDragCoefficient",
        rotor.rotorDragCoefficient, rotor.rotorDragCoefficient);
      getSdfParam<double>(rotorSDF, "rollingMomentCoefficient",
        rotor.rollingMomentCoefficient, rotor.rollingMomentCoefficient);
      getSdfParam<double>(rotorSDF, "maxRotVelocity",
        rotor.maxRotVelocity, rotor.maxRotVelocity);
      getSdfParam<double>(rotorSDF, "motorConstant",
        rotor.motorConstant, rotor.motorConstant);
      getSdfParam<double>(rotorSDF, "momentConstant",
        rotor.momentConstant, rotor.momentConstant);

      getSdfParam<double>(rotorSDF, "timeConstantUp",
        rotor.timeConstantUp, rotor.timeConstantUp);
      getSdfParam<double>(rotorSDF, "timeConstantDown",
        rotor.timeConstantDown, rotor.timeConstantDown);
      getSdfParam<double>(rotorSDF, "rotorVelocitySlowdownSim",
        rotor.rotorVelocitySlowdownSim, 1);

      getSdfParam<double>(rotorSDF, "frequencyCutoff",
        rotor.frequencyCutoff, rotor.frequencyCutoff);
      getSdfParam<double>(rotorSDF, "samplingRate",
        rotor.samplingRate, rotor.samplingRate);

      /* use gazebo::math::Filter */
      /// \TODO: compute cutoff frequency from timeConstantUp
      /// and rotor.timeConstantDown
      rotor.velocityFilter.SetFc(rotor.frequencyCutoff, rotor.samplingRate);
      // initialize filter to zero value
      rotor.velocityFilter.SetValue(0.0);
      // note to use this
      // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

      // Set the maximumForce on the joint and use Joint::SetVelocity.
      #undef USE_SET_VEL /// use SetVelocity or use pid force control
      #ifdef USE_SET_VEL
        #if GAZEBO_MAJOR_VERSION < 5
          rotor.joint->SetMaxForce(0, rotor.maxForce);
        #else
          // Joint::SetVelocity is deprecated in Gazebo 5.0
          // use SetForce with PID velocity feedback control.
          rotor.joint->SetParam("fmax", 0, 1000.0);
          // rotor.joint->SetParam("fmax", 0, rotor.maxForce);
        #endif
      #endif

      // Overload the PID parameters if they are available.
      getSdfParam<double>(rotorSDF, "vel_p_gain", rotor.pGain, rotor.pGain);
      getSdfParam<double>(rotorSDF, "vel_i_gain", rotor.iGain, rotor.iGain);
      getSdfParam<double>(rotorSDF, "vel_d_gain", rotor.dGain, rotor.dGain);
      getSdfParam<double>(rotorSDF, "vel_i_max", rotor.iMax, rotor.iMax);
      getSdfParam<double>(rotorSDF, "vel_i_min", rotor.iMin, rotor.iMin);
      getSdfParam<double>(rotorSDF, "vel_cmd_max", rotor.cmdMax, rotor.cmdMax);
      getSdfParam<double>(rotorSDF, "vel_cmd_min", rotor.cmdMin, rotor.cmdMin);

      // set PID parameters.
      rotor.pid.Init(rotor.pGain, rotor.iGain, rotor.dGain,
                     rotor.iMax, rotor.iMin,
                     rotor.cmdMax, rotor.cmdMin);

      // set pid initial command
      rotor.pid.SetCmd(0.0);

      this->rotors.push_back(rotor);
      rotorSDF = rotorSDF->GetNextElement("rotor");
    }
  }

  std::string imuLinkName;
  getSdfParam<std::string>(_sdf, "imuLinkName", imuLinkName, "iris/imu_link");
  this->imuLink = this->model->GetLink(imuLinkName);

  // Get sensors
  this->imuSensor =
    boost::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->model->GetWorld()->GetName()
        + "::" + this->model->GetScopedName()
        + "::iris::iris/imu_link::imu_sensor"));
  if (!this->imuSensor)
    gzerr << "imu_sensor not found\n" << "\n";

  // Controller time control.
  this->lastControllerUpdateTime = 0; // this->model->GetWorld()->GetSimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ArduCopterPlugin::Update, this, _1));

  // Initialize transport.
  // this->node = transport::NodePtr(new transport::Node());
  // this->node->Init();

  gzlog << "ArduCopter ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ArduCopterPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->GetSimTime();

  // Update the control surfaces and publish the new state.
  if (curTime > this->lastControllerUpdateTime)
  {
    this->SendState();
    this->GetMotorCommand();
    this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
  }

  this->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void ArduCopterPlugin::UpdatePIDs(double _dt)
{
  // Position PID for rotors
  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    double velTarget = this->rotors[i].multiplier * this->rotors[i].cmd /
         this->rotors[i].rotorVelocitySlowdownSim;
    #ifdef USE_SET_VEL
      #if GAZEBO_MAJOR_VERSION < 5
        this->rotors[i].joint->SetVelocity(0, velTarget);
      #else
       this->rotors[i].joint->SetParam("vel", 0, velTarget);
      #endif
    #else
      double vel = this->rotors[i].joint->GetVelocity(0);
      double error = vel - velTarget;
      double force = this->rotors[i].pid.Update(error, _dt);
      this->rotors[i].joint->SetForce(0, force);
    #endif
  }
}

/////////////////////////////////////////////////
void ArduCopterPlugin::GetMotorCommand()
{
  servo_packet pkt;
  /*
    we re-send the servo packet every 0.1 seconds until we get a
    reply. This allows us to cope with some packet loss to the FDM
  */
  while (this->recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
    // send_servos(input);
    usleep(100);
  }

  const double maxRPM = 1200.0;
  for (unsigned i = 0; i < this->rotors.size(); ++i)
  {
    this->rotors[i].cmd = maxRPM * pkt.motor_speed[i];
  }
}

/////////////////////////////////////////////////
void ArduCopterPlugin::SendState()
{
  // send_fdm
  fdm_packet pkt;

  pkt.timestamp = this->model->GetWorld()->GetSimTime().Double();

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration in body frame
  math::Vector3 linearAccel = this->imuSensor->LinearAcceleration();

  // rotate gravity into imu frame, subtract it
  // math::Vector3 gravity =
  //   this->model->GetWorld()->GetPhysicsEngine()->GetGravity();
  // linearAccel = linearAccel -
  //   this->imuLink.GetWorldPose().rot.RotateVectorReverse(gravity);

  // copy to pkt
  pkt.imu_linear_acceleration_xyz[0] = linearAccel.x;
  pkt.imu_linear_acceleration_xyz[1] = linearAccel.y;
  pkt.imu_linear_acceleration_xyz[2] = linearAccel.z;
  gzerr << "lin accel [" << linearAccel << "]\n";

  // get angular velocity in body frame
  math::Vector3 angularVel = this->imuSensor->AngularVelocity();
  // copy to pkt
  pkt.imu_angular_velocity_rpy[0] = angularVel.x;
  pkt.imu_angular_velocity_rpy[1] = angularVel.y;
  pkt.imu_angular_velocity_rpy[2] = angularVel.z;

  // get orientation with offset added
  // math::Quaternion worldQ = this->imuSensor->Orientation();
  // pkt.imu_orientation_quat[0] = worldQ.w;
  // pkt.imu_orientation_quat[1] = worldQ.x;
  // pkt.imu_orientation_quat[2] = worldQ.y;
  // pkt.imu_orientation_quat[3] = worldQ.z;

  // get inertial pose and velocity
  // position of the quadrotor in world frame
  // this position is used to calcualte bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from ardupilot helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the quadrotor in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  math::Pose gazeboToNED =
    math::Pose(math::Vector3(), math::Vector3(M_PI, 0, 0));

  // model world pose brings us to model, x-forward, y-left, z-up
  // adding gazeboToNED gets us to the x-forward, y-right, z-down
  math::Pose worldToModel = gazeboToNED + this->model->GetWorldPose();

  // get transform from world NED to Model frame
  math::Pose NEDToModel = worldToModel - gazeboToNED;

  gzerr << "ned to model [" << NEDToModel << "]\n";
  pkt.position_xyz[0] = NEDToModel.pos.x;  // N
  pkt.position_xyz[1] = NEDToModel.pos.y;  // E
  pkt.position_xyz[2] = NEDToModel.pos.z;  // D
  // This is the rotation from world NED frame to the quadrotor
  // frame.
  gzerr << "imu [" << worldToModel.rot.GetAsEuler() << "]\n";
  gzerr << "ned [" << gazeboToNED.rot.GetAsEuler() << "]\n";
  gzerr << "rot [" << NEDToModel.rot.GetAsEuler() << "]\n";
  pkt.imu_orientation_quat[0] = NEDToModel.rot.w;
  pkt.imu_orientation_quat[1] = NEDToModel.rot.x;
  pkt.imu_orientation_quat[2] = NEDToModel.rot.y;
  pkt.imu_orientation_quat[3] = NEDToModel.rot.z;

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  math::Vector3 velGazeboWorldFrame =
    this->model->GetLink()->GetWorldLinearVel();
  math::Vector3 velNEDFrame =
    gazeboToNED.rot.RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocity_xyz[0] = velNEDFrame.x;
  pkt.velocity_xyz[1] = velNEDFrame.y;
  pkt.velocity_xyz[2] = velNEDFrame.z;

  struct sockaddr_in sockaddr;
  this->make_sockaddr("127.0.0.1", 9003, sockaddr);
  ::sendto(this->handle, &pkt, sizeof(pkt), 0,
    (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}
