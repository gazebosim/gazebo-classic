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

#include <boost/program_options.hpp>

#include <gazebo/gui/qt.h>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/TopicPlot.hh>
#include <gazebo/gazebo_config.h>

namespace po = boost::program_options;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  po::options_description vDesc("Allowed options");
  vDesc.add_options()
    ("help,h", "print help message")
    ("period,p", po::value<unsigned int>(),
     "Period length in seconds. Default[10]");

  po::options_description hDesc("Hidden options");
  hDesc.add_options()
    ("topic", po::value<std::string>(), "Topic to plot")
    ("field", po::value<std::string>(), "Field to plot");

  po::options_description desc("Allowed options");
  desc.add(vDesc).add(hDesc);

  po::positional_options_description pDesc;
  pDesc.add("topic", 1).add("field", -1);
  po::variables_map vm;

  try
  {
    po::store(po::command_line_parser(_argc, _argv).options(desc).positional(
          pDesc).allow_unregistered().run(), vm);

    po::notify(vm);
  }
  catch(po::unknown_option& e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }


  if (vm.count("help"))
  {
    std::cerr << "Plot data from a published topic.\n\n"
              << "Usage: gzstats [options]\n"
              << desc << std::endl;
    return 1;
  }

  std::string topic, field;

  if (vm.count("topic"))
    topic = vm["topic"].as<std::string>();

  if (vm.count("field"))
    field = vm["field"].as<std::string>();

  unsigned int period = 10;
  if (vm.count("period"))
    period = vm["period"].as<unsigned int>();

  if (!gazebo::load())
  {
    printf("load error\n");
    return -1;
  }

  gazebo::run();

  QApplication *app = new QApplication(_argc, _argv);
  QFile file(":/style.qss");
  file.open(QFile::ReadOnly);
  QString styleSheet = QLatin1String(file.readAll());

  app->setStyleSheet(styleSheet);

  gazebo::gui::TopicPlot *plot = new gazebo::gui::TopicPlot(NULL);
  plot->Init(topic, field);
  plot->SetPeriod(period);
  plot->show();

  if (!gazebo::init())
  {
    gzerr << "Unable to initialize Gazebo\n";
    return -1;
  }

  app->exec();

  gazebo::fini();
}
