#include <lwr_peg_in_hole/peg_in_hole.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "peg_in_hole_node");

  PegInHole peg_in_hole;
  peg_in_hole.moveToStart();

  peg_in_hole.loadHolesLocation("plaque1.stl");

  peg_in_hole.moveAboveObjectHole("plaque1.stl", 0);
  peg_in_hole.moveAboveObjectHole("plaque1.stl", 1);
  peg_in_hole.moveAboveObjectHole("plaque1.stl", 2);

  ros::shutdown();
  return 0;
}