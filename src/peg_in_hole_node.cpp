#include <lwr_peg_in_hole/peg_in_hole.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "peg_in_hole_node");

  PegInHole peg_in_hole;
  peg_in_hole.moveToStart();

  peg_in_hole.loadHolesLocation("holes_board");

  peg_in_hole.moveAboveObjectHole("holes_board", 0);
  peg_in_hole.moveAboveObjectHole("holes_board", 1);
  peg_in_hole.moveAboveObjectHole("holes_board", 2);

  ros::shutdown();
  return 0;
}