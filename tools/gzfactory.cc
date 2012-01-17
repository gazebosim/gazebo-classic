#include <transport/transport.h>
#include <fstream>
#include <string>

using namespace gazebo;

void help()
{
  std::cout << "This tool spawns models into a running Gazebo simulation.\n\n"
            << "  gzfactory <world_name> <model_filename>\n"
            << "    world_name     : Name of the world in which to spawn "
            << "the model.\n"
            << "    modle_filename : Filename of the SDF model.\n\n";
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    help();
    return -1;
  }

  std::ifstream ifs(argv[2]);
  if (!ifs)
  {
    printf("Error: Unable to open file[%s]\n", argv[2]);
    return -1;
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));

  transport::init();
  transport::NodePtr node(new transport::Node());

  node->Init(argv[1]);

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  transport::run();

  msgs::Factory msg;
  msg.set_sdf(content);
  pub->Publish(msg, true);

  transport::fini();
  printf("Spawn complete\n");
}
