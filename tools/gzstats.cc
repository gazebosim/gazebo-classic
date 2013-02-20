/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <signal.h>
#include <google/protobuf/message.h>
#include <boost/thread.hpp>

#include "transport/Transport.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"

#include "common/Animation.hh"
#include "common/KeyFrame.hh"

#include "gazebo_config.h"

using namespace gazebo;

std::list<common::Time> simTimes, realTimes;

boost::mutex mutex;
boost::condition_variable condition;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "This tool displays statistics about a running Gazebo world.\n"
            << "  Usage: gzstats <world_name>\n"
            << "    <world_name> : Output statistics for the given world. "
            << " (Default = \"default\")\n"
            << "    help         : This help text\n";
}

/////////////////////////////////////////////////
void cb(ConstWorldStatisticsPtr &_msg)
{
  double percent = 0;
  char paused;
  common::Time simTime  = msgs::Convert(_msg->sim_time());
  common::Time realTime = msgs::Convert(_msg->real_time());

  simTimes.push_back(msgs::Convert(_msg->sim_time()));
  if (simTimes.size() > 20)
    simTimes.pop_front();

  realTimes.push_back(msgs::Convert(_msg->real_time()));
  if (realTimes.size() > 20)
    realTimes.pop_front();

  common::Time simAvg, realAvg;
  std::list<common::Time>::iterator simIter, realIter;
  simIter = ++(simTimes.begin());
  realIter = ++(realTimes.begin());
  while (simIter != simTimes.end() && realIter != realTimes.end())
  {
    simAvg += ((*simIter) - simTimes.front());
    realAvg += ((*realIter) - realTimes.front());
    ++simIter;
    ++realIter;
  }

  // Prevent divide by zero
  if (realAvg <= 0)
    return;

  simAvg = simAvg / realAvg;

  if (simAvg > 0)
    percent = simAvg.Double();
  else
    percent = 0;


  if (_msg->paused())
    paused = 'T';
  else
    paused = 'F';

  printf("Factor[%4.2f] SimTime[%4.2f] RealTime[%4.2f] Paused[%c]\n",
      percent, simTime.Double(), realTime.Double(), paused);
  fflush(stdout);
}

//////////////////////////////////////////////////
void SignalHandler(int /*dummy*/)
{
  boost::mutex::scoped_lock lock(mutex);
  condition.notify_all();
  return;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  std::string worldName = "default";
  if (argc > 1)
    worldName = argv[1];

  if (worldName == "help")
  {
    help();
    return -1;
  }

  transport::init();

  transport::NodePtr node(new transport::Node());

  node->Init(worldName);

  std::string topic = "~/world_stats";

  transport::SubscriberPtr sub = node->Subscribe(topic, cb);
  transport::run();

  boost::mutex::scoped_lock lock(mutex);
  condition.wait(lock);

  transport::fini();

  return 0;
}
