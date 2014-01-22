/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;
class PhysicsTest : public ServerFixture
{
};


TEST_F(PhysicsTest, State)
{
  srand(time(NULL));
  int seed = time(NULL);
  {
    // intentionally break the joint using Link::SetWorldPose
    // let it conflict with Physics pose updates and make sure
    // internal model state stays consistent

    Load("worlds/empty.world");
    physics::WorldPtr world = physics::get_world("default");
    world->SetPaused(true);
    EXPECT_TRUE(world != NULL);

    physics::WorldState worldState = world->GetState();
    physics::ModelState modelState = worldState.GetModelState(0);
    physics::LinkState linkState = modelState.GetLinkState(0);


    {
      msgs::Factory msg;

      std::ostringstream newModelStr;

      math::Pose pose(0, 0, 3, 0, 0, 0);
      math::Vector3 size(1.0, 0.1, 0.1);
      newModelStr
        << "<gazebo version='" << SDF_VERSION << "'>\n"
        << "  <model name='model_1'>\n"
        << "    <pose>" << pose << "</pose>\n"
        << "    <link name='link_1'>\n"
        << "      <pose>0 0 0 0 0 0</pose>\n"
        << "      <inertial>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <mass>1</mass>\n"
        << "        <inertia>\n"
        << "          <ixx>1</ixx>\n"
        << "          <ixy>0</ixy>\n"
        << "          <ixz>0</ixz>\n"
        << "          <iyy>1</iyy>\n"
        << "          <iyz>0</iyz>\n"
        << "          <izz>1</izz>\n"
        << "        </inertia>\n"
        << "      </inertial>\n"
        << "      <collision name ='collision'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "      </collision>\n"
        << "      <visual name ='visual'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "        <material><script>Gazebo/Grey</script></material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "    <link name='link_2'>\n"
        << "      <pose>1 0 0 0 0 0</pose>\n"
        << "      <inertial>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <mass>1</mass>\n"
        << "        <inertia>\n"
        << "          <ixx>1</ixx>\n"
        << "          <ixy>0</ixy>\n"
        << "          <ixz>0</ixz>\n"
        << "          <iyy>1</iyy>\n"
        << "          <iyz>0</iyz>\n"
        << "          <izz>1</izz>\n"
        << "        </inertia>\n"
        << "      </inertial>\n"
        << "      <collision name ='collision'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "      </collision>\n"
        << "      <visual name ='visual'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "        <material><script>Gazebo/Grey</script></material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "    <joint name='joint_01' type='revolute'>\n"
        << "      <parent>world</parent>\n"
        << "      <child>link_1</child>\n"
        << "      <axis>\n"
        << "        <xyz>1 1 0</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <joint name='joint_12' type='revolute'>\n"
        << "      <parent>link_1</parent>\n"
        << "      <child>link_2</child>\n"
        << "      <axis>\n"
        << "        <xyz>1 -1 0</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <link name='link_3'>\n"
        << "      <pose>2 0 0 0 0 " << 0.5*M_PI << "</pose>\n"
        << "      <inertial>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <mass>1</mass>\n"
        << "        <inertia>\n"
        << "          <ixx>1</ixx>\n"
        << "          <ixy>0</ixy>\n"
        << "          <ixz>0</ixz>\n"
        << "          <iyy>1</iyy>\n"
        << "          <iyz>0</iyz>\n"
        << "          <izz>1</izz>\n"
        << "        </inertia>\n"
        << "      </inertial>\n"
        << "      <collision name ='collision'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "      </collision>\n"
        << "      <visual name ='visual'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "        <material><script>Gazebo/Grey</script></material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "    <joint name='joint_23' type='revolute'>\n"
        << "      <parent>link_2</parent>\n"
        << "      <child>link_3</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <link name='link_4'>\n"
        << "      <pose>2 1 0 0 0 " << M_PI << "</pose>\n"
        << "      <inertial>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <mass>1</mass>\n"
        << "        <inertia>\n"
        << "          <ixx>1</ixx>\n"
        << "          <ixy>0</ixy>\n"
        << "          <ixz>0</ixz>\n"
        << "          <iyy>1</iyy>\n"
        << "          <iyz>0</iyz>\n"
        << "          <izz>1</izz>\n"
        << "        </inertia>\n"
        << "      </inertial>\n"
        << "      <collision name ='collision'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "      </collision>\n"
        << "      <visual name ='visual'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "        <material><script>Gazebo/Grey</script></material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "    <joint name='joint_34' type='revolute'>\n"
        << "      <parent>link_3</parent>\n"
        << "      <child>link_4</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <link name='link_5'>\n"
        << "      <pose>1 1 0 0 0 " << 1.5*M_PI << "</pose>\n"
        << "      <inertial>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <mass>1</mass>\n"
        << "        <inertia>\n"
        << "          <ixx>1</ixx>\n"
        << "          <ixy>0</ixy>\n"
        << "          <ixz>0</ixz>\n"
        << "          <iyy>1</iyy>\n"
        << "          <iyz>0</iyz>\n"
        << "          <izz>1</izz>\n"
        << "        </inertia>\n"
        << "      </inertial>\n"
        << "      <collision name ='collision'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "      </collision>\n"
        << "      <visual name ='visual'>\n"
        << "        <pose>0.5 0 0 0 0 0</pose>\n"
        << "        <geometry>\n"
        << "          <box>\n"
        << "            <size>" << size.x << " " << size.y << " "
        << size.z << "</size>\n"
        << "          </box>\n"
        << "        </geometry>\n"
        << "        <material><script>Gazebo/Grey</script></material>\n"
        << "      </visual>\n"
        << "    </link>\n"
        << "    <joint name='joint_45' type='revolute'>\n"
        << "      <parent>link_4</parent>\n"
        << "      <child>link_5</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <joint name='joint_52' type='revolute'>\n"
        << "      <parent>link_5</parent>\n"
        << "      <child>link_2</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "  </model>\n"
        << "</gazebo>\n";

      msg.set_sdf(newModelStr.str());

      transport::PublisherPtr factoryPub
        = this->node->Advertise<msgs::Factory>("~/factory");
      this->factoryPub->Publish(msg);
    }
    physics::ModelPtr model = world->GetModel("model_1");
    while (!model)
    {
      model = world->GetModel("model_1");
      gzdbg << "waiting for model_1 to spawn\n";
      sleep(1);
    }
    world->SetPaused(false);


    double start_time;
    double start_wall_time;
    double test_duration;
    double pub_rate;
    double last_update_time;
    double elapsed_wall_time;


    physics::JointPtr joint_01 = model->GetJoint("model_1::joint_01");
    physics::JointPtr joint_12 = model->GetJoint("model_1::joint_12");
    physics::JointPtr joint_23 = model->GetJoint("model_1::joint_23");
    physics::JointPtr joint_34 = model->GetJoint("model_1::joint_34");
    physics::JointPtr joint_45 = model->GetJoint("model_1::joint_45");
    physics::JointPtr joint_52 = model->GetJoint("model_1::joint_52");

    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 10;
    pub_rate = 10.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Joint::SetAngle at ["
      << pub_rate << "] Hz.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        // gzdbg << "setting link poses without violation\n";
        // double cur_time = world->GetSimTime().Double();
        joint_01->SetAngle(0, 0.1);
        joint_12->SetAngle(0, 0.1);
        joint_23->SetAngle(0, 0.1);
        joint_34->SetAngle(0, 0.1);
        joint_45->SetAngle(0, 0.1);
        joint_52->SetAngle(0, 0.1);

        joint_01->SetAngle(0, 0.2);
        joint_12->SetAngle(0, 0.2);
        joint_23->SetAngle(0, 0.2);
        joint_34->SetAngle(0, 0.2);
        joint_45->SetAngle(0, 0.2);
        joint_52->SetAngle(0, 0.2);
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";




    world->EnablePhysicsEngine(false);

    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 20;
    pub_rate = 10.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Joint::SetAngle at ["
      << pub_rate << "] Hz with real time duration.\n";
    last_update_time = start_wall_time;
    while (world->GetRealTime().Double() < start_wall_time + test_duration)
      if (world->GetRealTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetRealTime().Double();
        joint_01->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        joint_12->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        joint_23->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        joint_34->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        joint_45->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        joint_52->SetAngle(0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        sleep(1);
      }
    test_duration = world->GetSimTime().Double() - start_time;
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";


    world->EnablePhysicsEngine(true);
















    physics::LinkPtr link_1 = model->GetLink("link_1");
    physics::LinkPtr link_2 = model->GetLink("link_2");
    physics::LinkPtr link_3 = model->GetLink("link_3");
    physics::LinkPtr link_4 = model->GetLink("link_4");
    physics::LinkPtr link_5 = model->GetLink("link_5");

    EXPECT_TRUE(link_1 != NULL);

    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 10;
    pub_rate = 2.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing SetWorld Pose at ["
      << pub_rate << "] Hz without constraint violation.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        // gzdbg << "setting link poses without violation\n";
        // double cur_time = world->GetSimTime().Double();
        last_update_time = world->GetSimTime().Double();
        link_1->SetWorldPose(math::Pose(0,    0, 3, 0,        0, 0));
        link_2->SetWorldPose(math::Pose(1.00, 0, 3, 0,        0, 0));
        link_3->SetWorldPose(math::Pose(2.00, 0, 3, 0, 0.5*M_PI, 0));
        link_4->SetWorldPose(math::Pose(2.00, 1, 3, 0, 1.0*M_PI, 0));
        link_5->SetWorldPose(math::Pose(1.00, 1, 3, 0, 1.5*M_PI, 0));
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";

    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 10;
    pub_rate = 1.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing SetWorld Pose at ["
      << pub_rate << "] Hz without constraint violation.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        // gzdbg << "setting link poses without violation\n";
        // double cur_time = world->GetSimTime().Double();
        link_1->SetWorldPose(math::Pose(0,    0, 3, 0, 0, 0));
        link_2->SetWorldPose(math::Pose(1.00, 0, 3, 0, 0, 0));
        link_3->SetWorldPose(math::Pose(2.00, 0, 3, 0, 0.5*M_PI, 0));
        link_4->SetWorldPose(math::Pose(2.00, 1, 3, 0, 1.0*M_PI, 0));
        link_5->SetWorldPose(math::Pose(1.00, 1, 3, 0, 1.5*M_PI, 0));
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";




    // set random pose within joint limit
    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 20;
    pub_rate = 2.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing SetWorld Pose at ["
      << pub_rate << "] Hz with less constraint violation.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      // if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      math::Pose p;
      p = math::Pose(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_1->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_2->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 2.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/
          static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/
          static_cast<double>(RAND_MAX) + 0.5*M_PI,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_3->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 2.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/
          static_cast<double>(RAND_MAX) + M_PI,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_4->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(seed))/
          static_cast<double>(RAND_MAX) + 1.5*M_PI,
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_5->SetWorldPose(p);

      math::Vector3 v;
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_1->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_1->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_2->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_2->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_3->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_3->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_4->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_4->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_5->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
      link_5->SetLinearVel(v);
    }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";



    // set random pose outside of joint limit
    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 20;
    pub_rate = 1000.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing SetWorld Pose at ["
      << pub_rate << "] Hz with more random constraint violation.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        math::Pose p;
        p = math::Pose(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 3.0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.57079,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_1->SetWorldPose(p);
        p = math::Pose(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 2.0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.57079,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_2->SetWorldPose(p);
        p = math::Pose(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.57079,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_3->SetWorldPose(p);
        p = math::Pose(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.57079,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_4->SetWorldPose(p);
        p = math::Pose(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.0,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/
            static_cast<double>(RAND_MAX) + 1.57079,
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_5->SetWorldPose(p);

        math::Vector3 v;
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_1->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_1->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_2->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_2->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_3->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_3->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_4->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_4->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_5->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_5->SetLinearVel(v);
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";



    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 10;
    pub_rate = 1000.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Set*Vel at ["
      << pub_rate << "] Hz with random velocities.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        // gzdbg << "setting link poses with violation\n";
        math::Vector3 v;
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_1->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_1->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_2->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_2->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_3->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_3->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_4->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_4->SetLinearVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_5->SetAngularVel(v);
        v = math::Vector3(
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX),
            static_cast<double>(rand_r(seed))/static_cast<double>(RAND_MAX));
        link_5->SetLinearVel(v);
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";

    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 20;
    pub_rate = 500.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Set*Vel at ["
      << pub_rate << "] Hz with velocity decay.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        // gzdbg << "setting link poses with violation\n";
        link_1->SetAngularVel(link_1->GetWorldAngularVel() * 0.999);
        link_1->SetLinearVel(link_1->GetWorldLinearVel()  * 0.999);
        link_2->SetAngularVel(link_2->GetWorldAngularVel() * 0.999);
        link_2->SetLinearVel(link_2->GetWorldLinearVel()  * 0.999);
        link_3->SetAngularVel(link_3->GetWorldAngularVel() * 0.999);
        link_3->SetLinearVel(link_3->GetWorldLinearVel()  * 0.999);
        link_4->SetAngularVel(link_4->GetWorldAngularVel() * 0.999);
        link_4->SetLinearVel(link_4->GetWorldLinearVel()  * 0.999);
        link_5->SetAngularVel(link_5->GetWorldAngularVel() * 0.999);
        link_5->SetLinearVel(link_5->GetWorldLinearVel()  * 0.999);
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";

    EXPECT_EQ(link_3->GetWorldPose(),
        math::Pose(0.292968, 0.612084, 1.43649, -2.07141, 1.50881, -1.19487));
    Unload();
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
