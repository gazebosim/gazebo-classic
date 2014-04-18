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
#include <boost/program_options.hpp>
#include <signal.h>
#include <google/protobuf/message.h>
#include <boost/thread.hpp>

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyFrame.hh"

#include "gazebo/gazebo_config.h"

namespace po = boost::program_options;
using namespace gazebo;

std::list<common::Time> simTimes, realTimes;

boost::mutex mutex;
boost::condition_variable condition;

bool g_plot;

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

  if (g_plot)
  {
    static bool first = true;
    if (first)
    {
      printf("# real-time factor (percent), simtime (sec), realtime (sec), "
             "paused (T or F)\n");
      first = false;
    }
    printf("%4.2f, %16.6f, %16.6f, %c\n",
        percent, simTime.Double(), realTime.Double(), paused);
    fflush(stdout);
  }
  else
    printf("Factor[%4.2f] SimTime[%4.2f] RealTime[%4.2f] Paused[%c]\n",
        percent, simTime.Double(), realTime.Double(), paused);
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

  po::options_description desc("Options");
  desc.add_options()
    ("help,h", "Print help message.")
    ("plot,p", "Output comma-separated values, useful for processing and "
               "plotting.")
    ("world-name,w", po::value<std::string>(), "The Gazebo world to monitor.");
  po::variables_map vm;

  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  }
  catch(po::unknown_option& e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  po::notify(vm);

  if (vm.count("help"))
  {
    std::cerr << "gzstats -- DEPRECATED(see 'gz help sdf')\n\n";

    std::cerr << "`gzstats` [options]\n\n";

    std::cerr << "This tool displays statistics about a running "
      "Gazebo world.\n\n";

    std::cerr << desc << "\n";

    std::cerr << "See also:\n"
      << "Examples and more information can be found at: "
      << "http://gazebosim.org/wiki/Tools#World_Statistics\n";

    return 1;
  }

  std::string worldName;
  if (vm.count("world-name"))
  {
    worldName = vm["world-name"].as<std::string>();
  }

  if (vm.count("plot"))
  {
    g_plot = true;
  }

  if (transport::init())
  {
    transport::NodePtr node(new transport::Node());

    node->Init(worldName);

    std::string topic = "~/world_stats";

    transport::SubscriberPtr sub = node->Subscribe(topic, cb);
    transport::run();

    boost::mutex::scoped_lock lock(mutex);
    condition.wait(lock);
  }

  transport::fini();

  return 0;
}
