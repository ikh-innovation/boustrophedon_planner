#include <ros/ros.h>
#include "boustrophedon_server/boustrophedon_planner_server.h"


BoustrophedonPlannerServer* server;

void callback(boustrophedon_server::BoustrophedonParametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Requested");

  server->setParameters(config);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_node");
  server = new BoustrophedonPlannerServer;
  dynamic_reconfigure::Server<boustrophedon_server::BoustrophedonParametersConfig> dynamic_reconfigure_server;
  dynamic_reconfigure::Server<boustrophedon_server::BoustrophedonParametersConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  dynamic_reconfigure_server.setCallback(f);

  ros::spin();

  return 0;
}
