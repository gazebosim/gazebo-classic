#include <libplayerc++/playerc++.h>

int main(int /*argc*/, char ** /*argv*/)
{
  PlayerCc::PlayerClient client(PlayerCc::PLAYER_HOSTNAME,
                                PlayerCc::PLAYER_PORTNUM);
  PlayerCc::SimulationProxy sp(&client, 0);
  PlayerCc::Position2dProxy pp(&client, 0);

  sp.SetPose2d((char*)("box_model1"),1,1,0);
  sp.SetPose3d((char*)("box_model1"),-1,-1,1,0,0,0.707);

  return 1;
}
