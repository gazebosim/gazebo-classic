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
  {
    // intentionally break the joint using Link::SetWorldPose
    // let it conflict with Physics pose updates and make sure
    // internal model state stays consistent

    Load("worlds/empty.world");
    physics::WorldPtr world = physics::get_world("default");
    world->SetPaused(true);
    EXPECT_TRUE(world != NULL);

    world->GetPhysicsEngine()->SetGravity(math::Vector3(0, 0, 0));

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

        << "    <link name='link_2a'>\n"  // opposite of link_5
        << "      <pose>1 -1 0 0 0 " << 0.5*M_PI << "</pose>\n"
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
        << "    <joint name='joint_2a2' type='revolute'>\n"
        << "      <parent>link_2a</parent>\n"
        << "      <child>link_2</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"

        << "    <link name='link_2b'>\n"  // second loop start
        << "      <pose>1 -1 0 0 0 " << 0.0*M_PI << "</pose>\n"
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
        << "    <joint name='joint_2a2b' type='revolute'>\n"
        << "      <parent>link_2a</parent>\n"
        << "      <child>link_2b</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"

        << "    <link name='link_3a'>\n"  // second branch
        << "      <pose>2 -1 0 0 0 " << -0.5*M_PI << "</pose>\n"
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
        << "    <joint name='joint_2a3a' type='revolute'>\n"
        << "      <parent>link_2b</parent>\n"
        << "      <child>link_3a</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"

        << "    <link name='link_4a'>\n"  // second branch
        << "      <pose>2 -2 0 0 0 " << -1.0*M_PI << "</pose>\n"
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
        << "    <joint name='joint_3a4a' type='revolute'>\n"
        << "      <parent>link_3a</parent>\n"
        << "      <child>link_4a</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"

        << "    <link name='link_5a'>\n"  // second branch
        << "      <pose>1 -2 0 0 0 " << 0.5*M_PI << "</pose>\n"
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
        << "    <joint name='joint_4a5a' type='revolute'>\n"
        << "      <parent>link_4a</parent>\n"
        << "      <child>link_5a</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
        << "        <limit>\n"
        << "          <upper>0.7071</upper>\n"
        << "          <lower>-0.7071</lower>\n"
        << "        </limit>\n"
        << "      </axis>\n"
        << "    </joint>\n"
        << "    <joint name='joint_5a2b' type='revolute'>\n"  // loop closer
        << "      <parent>link_5a</parent>\n"
        << "      <child>link_2b</child>\n"
        << "      <axis>\n"
        << "        <xyz>0 0 1</xyz>\n"
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

    sleep(5);

    world->EnablePhysicsEngine(false);

    physics::ModelPtr model = world->GetModel("model_1");
    while (!model)
    {
      model = world->GetModel("model_1");
      gzdbg << "waiting for model_1 to spawn\n";
      sleep(1);
    }
    world->SetPaused(false);

    physics::JointPtr joint_01 = model->GetJoint("model_1::joint_01");
    physics::JointPtr joint_12 = model->GetJoint("model_1::joint_12");
    physics::JointPtr joint_23 = model->GetJoint("model_1::joint_23");
    physics::JointPtr joint_34 = model->GetJoint("model_1::joint_34");
    physics::JointPtr joint_45 = model->GetJoint("model_1::joint_45");
    physics::JointPtr joint_52 = model->GetJoint("model_1::joint_52");
    physics::JointPtr joint_2a2 = model->GetJoint("model_1::joint_2a2");
    physics::JointPtr joint_2a2b = model->GetJoint("model_1::joint_2a2b");
    physics::JointPtr joint_2a3a = model->GetJoint("model_1::joint_2a3a");
    physics::JointPtr joint_3a4a = model->GetJoint("model_1::joint_3a4a");
    physics::JointPtr joint_4a5a = model->GetJoint("model_1::joint_4a5a");
    physics::JointPtr joint_5a2b = model->GetJoint("model_1::joint_5a2b");

    EXPECT_TRUE(joint_01   != NULL);
    EXPECT_TRUE(joint_12   != NULL);
    EXPECT_TRUE(joint_23   != NULL);
    EXPECT_TRUE(joint_34   != NULL);
    EXPECT_TRUE(joint_45   != NULL);
    EXPECT_TRUE(joint_52   != NULL);
    EXPECT_TRUE(joint_2a2  != NULL);
    EXPECT_TRUE(joint_2a2b != NULL);
    EXPECT_TRUE(joint_2a3a != NULL);
    EXPECT_TRUE(joint_3a4a != NULL);
    EXPECT_TRUE(joint_4a5a != NULL);
    EXPECT_TRUE(joint_5a2b != NULL);


    double start_time;
    double start_wall_time;
    double test_duration;
    double pub_rate;
    double last_update_time;
    double elapsed_wall_time;

    srand_r(seed)(time(NULL));
    int seed = time(NULL);














    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 5;
    pub_rate = 1.0;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Joint::SetAngle at ["
      << pub_rate << "] Hz in real time, to all joint, should diverge.\n";
    last_update_time = start_wall_time;
    while (world->GetRealTime().Double() < start_wall_time + test_duration)
      if (world->GetRealTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetRealTime().Double();

        int n = model->GetJointCount();
        for (int i = 0; i < n; ++i)
          model->GetJoint(i)->SetAngle(0,
              0.1*static_cast<double>(rand_r(seed)())/
              static_cast<double>(RAND_MAX));
      }
    test_duration = world->GetSimTime().Double() - start_time;
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";









    world->EnablePhysicsEngine(true);


    /* set all link velocities to 0
       std::vector<physics::LinkPtr> links = model->GetAllLinks();
       for (unsigned i = 0; i < links.size(); ++i)
       {
       links[i]->SetLinearVel(math::Vector3(0, 0, 0));
       links[i]->SetAngularVel(math::Vector3(0, 0, 0));
       } */


    start_time = world->GetSimTime().Double();
    start_wall_time = world->GetRealTime().Double();
    test_duration = 20;
    pub_rate = 0.5;
    gzdbg << " -------------------------------------------------------------\n";
    gzdbg << " Publishing Joint::SetAngle at ["
      << pub_rate << "] Hz to non-loop joints, should have no violation.\n";
    last_update_time = start_time;
    while (world->GetSimTime().Double() < start_time + test_duration)
      if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
      {
        last_update_time = world->GetSimTime().Double();
        double a = 0.50*(2.0*
            static_cast<double>(rand_r(seed)())/
            static_cast<double>(RAND_MAX) - 1.0);
        joint_01->SetAngle(0, a);
        // joint_12->SetAngle(0, a);
        // joint_23->SetAngle(0, a);
        // joint_2a2->SetAngle(0, a);
      }
    elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
    gzdbg << "  elapsed sim time [" << test_duration
      << "] elapsed wall time [" << elapsed_wall_time
      << "] sim performance [" << test_duration / elapsed_wall_time
      << "]\n";

    // EXPECT_EQ(joint_5a2b->GetAngle(0).Radian(), 0.0);
    Unload();
  }
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
