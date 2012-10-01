/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "physics/physics.hh"

using namespace gazebo;
class PhysicsTest : public ServerFixture
{
};


TEST_F(PhysicsTest, State)
{
  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  physics::WorldState worldState = world->GetState();
  physics::ModelState modelState = worldState.GetModelState(0);
  physics::LinkState linkState = modelState.GetLinkState(0);


  {
    msgs::Factory msg;

    std::ostringstream newModelStr;

    math::Pose pose(0, 0, 0, 0, 0, 0);
    math::Vector3 size(1.0, 0.1, 0.1);

    newModelStr
      << "<gazebo version ='1.2'>\n"
      << "  <model name='model_1'>\n"
      << "    <pose>0 0 1 0 0 0</pose>\n"
      << "    <link name='link_1'>\n"
      << "      <pose>0 0 0 0 0 0</pose>\n"
      << "      <inertial>\n"
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
      << "        <xyz>0 0 1</xyz>\n"
      << "        <limit>\n"
      << "          <upper>0</upper>\n"
      << "          <lower>0</lower>\n"
      << "        </limit>\n"
      << "      </axis>\n"
      << "    </joint>\n"
      << "    <joint name='joint_12' type='revolute'>\n"
      << "      <parent>link_1</parent>\n"
      << "      <child>link_2</child>\n"
      << "      <axis>\n"
      << "        <xyz>0 0 1</xyz>\n"
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
  while(!model)
  {
    model = world->GetModel("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    sleep(1);
  }

  physics::LinkPtr link_1 = model->GetLink("link_1");
  physics::LinkPtr link_2 = model->GetLink("link_2");
  double start_time = world->GetSimTime().Double();
  while(world->GetSimTime().Double() < start_time + 30)
  {
    // gzdbg << "setting link poses without violation\n";
    // double cur_time = world->GetSimTime().Double();
    link_1->SetWorldPose(math::Pose(0,    0, 1, 0, 0, 0));
    link_2->SetWorldPose(math::Pose(1.00, 0, 1, 0, 0, 0));
    sleep(0.5);
  }

  // world->EnablePhysicsEngine(false);

  // FIXME: below fails, but why?
  start_time = world->GetSimTime().Double();
  while(world->GetSimTime().Double() < start_time + 100)
  {
    gzdbg << "setting link poses with violation\n";
    link_1->SetWorldPose(math::Pose(0,    0, 1, 0, 0, 0));
    link_2->SetWorldPose(math::Pose(2.00, 0, 1, 0, 0, 1.57079));
    sleep(0.5);
  }

  Unload();
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
