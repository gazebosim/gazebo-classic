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
#include <transport/transport.h>
#include <boost/program_options.hpp>
#include <fstream>
#include <string>

using namespace gazebo;
namespace po = boost::program_options;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "This tool spawns models into a running Gazebo simulation.\n\n"
            << "  gzfactory <world_name> <model_filename>\n"
            << "    world_name     : Name of the world in which to spawn "
            << "the model.\n"
            << "    modle_filename : Filename of the SDF model.\n\n";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  po::options_description v_desc("Allowd options");
  v_desc.add_options()
    ("help,h", "produce this help message")
    ("sdf,f", po::value<std::string>(), "SDF model file")
    ("pose-x,x", po::value<double>(), "set model x position.")
    ("pose-x,y", po::value<double>(), "set model y position.")
    ("pose-x,z", po::value<double>(), "set model z positione.")
    ("pose-x,r", po::value<double>(), "set model roll orientation in radians.")
    ("pose-x,p", po::value<double>(), "set model pitch orientation in radians.")
    ("pose-x,y", po::value<double>(), "set model yaw orientation in radians.");

  po::options_description h_desc("Hidden options");
  h_desc.add_options()
    ("command", po::value<std::string>(), "<spawn|delete>")
    ("model-name", po::value<std::string>(), "overwrite SDF name of model.");
  
  po::options_description desc("Allowed options");
  desc.add(v_desc).add(h_desc);

  po::positional_options_description p_desc;
  p_desc.add("command", 1);
  p_desc.add("model-name", 2);

  po::variables_map vm;
  po::store(po::command_line_parser(argc,
        argv).options(desc).positional(p_desc).run(), vm);
  po::notify(vm);

  if (vm.count("help") || argc < 2)
  {
    help();
    std::cout << v_desc << "\n";
    return -1;
  }

  return 0;

  std::ifstream ifs;
  if (vm.count("input-filename"))
    ifs.open(vm["input-filename"].as<std::string>().c_str());


  if (!ifs)
  {
    printf("Error: Unable to open file[%s]\n", argv[2]);
    return -1;
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));

  transport::init();

  transport::run();

  transport::NodePtr node(new transport::Node());
  node->Init(argv[1]);

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  pub->WaitForConnection();

  msgs::Factory msg;
  msg.set_sdf(content);
  pub->Publish(msg, true);

  transport::fini();
}
