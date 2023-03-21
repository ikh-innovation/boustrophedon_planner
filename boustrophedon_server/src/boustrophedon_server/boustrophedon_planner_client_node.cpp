#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

// server has a service to convert StripingPlan to Path, but all it does it call this method
bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
{
  path.header.frame_id = striping_plan.header.frame_id;
  path.header.stamp = striping_plan.header.stamp;

  path.poses.clear();

  // path.poses.resize(striping_plan.points.size());
  // std::transform(striping_plan.points.begin(), striping_plan.points.end(), path.poses.begin(),
  //                [&](const boustrophedon_msgs::StripingPoint& point) {
  //                  geometry_msgs::PoseStamped pose;
  //                  pose.header.frame_id = striping_plan.header.frame_id;
  //                  pose.header.stamp = striping_plan.header.stamp;
  //                  pose.pose.position = point.point;
  //                  pose.pose.orientation.x = 0.0;
  //                  pose.pose.orientation.y = 0.0;
  //                  pose.pose.orientation.z = 0.0;
  //                  pose.pose.orientation.w = 1.0;
  //                  return pose;
  //                });

  for (std::size_t i = 0; i < striping_plan.points.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = striping_plan.header.frame_id;
    pose.header.stamp = striping_plan.header.stamp;
    pose.pose.position = striping_plan.points[i].point;

    if (i < striping_plan.points.size() - 1)
    {
      double dx = striping_plan.points[i + 1].point.x - striping_plan.points[i].point.x;
      double dy = striping_plan.points[i + 1].point.y - striping_plan.points[i].point.y;
      double dz = striping_plan.points[i + 1].point.z - striping_plan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
    }
    else
    {
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
    }

    path.poses.push_back(pose);
  }

  return true;
}

bool got_initial_pose = false;
geometry_msgs::PoseStamped initial_pose;
boustrophedon_msgs::PlanMowingPathGoal goal;
int point_num {0};


void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr init_pose)
{
  // Uncomment the following if you want the initial pose to be a point of the polygon
  //----------------------------------------------------------------------------------
  // if (point_num > 1)
  // {
  //   ROS_INFO_STREAM("Got final point which corresponds to initial position.");
  //   initial_pose.header = init_pose->header;
  //   initial_pose.pose = init_pose->pose.pose;
  //   got_initial_pose = true;

  //   goal.property.polygon.points.resize(point_num+1);
  //   goal.property.polygon.points[point_num].x = init_pose->pose.pose.position.x;
  //   goal.property.polygon.points[point_num].y = init_pose->pose.pose.position.y;
  //   ROS_INFO("Total point number is : %i", point_num+1);
  //   point_num = 0 ;
  // }
  // else
  // {
  //   ROS_INFO("You have only entered %i point(s). Clearing point list and starting over...", point_num+1);
  //   point_num = 0 ;
  // }
  //----------------------------------------------------------------------------------

  if (point_num > 1)
  {
    ROS_INFO_STREAM("Got initial position.");
    initial_pose.header = init_pose->header;
    initial_pose.pose = init_pose->pose.pose;
    got_initial_pose = true;
    ROS_INFO("Total point number is : %i", point_num+1);
    point_num = 0 ;
  }
  else
  {
    ROS_INFO("You have only entered %i point(s). Clearing point list and starting over...", point_num+1);
    point_num = 0 ;
  }
}

void addPolygonPointCallback(const geometry_msgs::PointStampedConstPtr new_point)
{
  ROS_INFO_STREAM("Got new point");
  goal.property.polygon.points.resize(point_num+1);
  goal.property.polygon.points[point_num].x = new_point->point.x;
  goal.property.polygon.points[point_num].y = new_point->point.y;
  point_num++;
  ROS_INFO("Total point number is : %i", point_num);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_client");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client("plan_path",
                                                                                 true);  // server name and spin thread

  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/result_path", 1, true);
  ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);
  ros::Subscriber init_pose =
      n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);

  ros::Subscriber add_points =
      n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, addPolygonPointCallback);
  ros::Rate loop_rate(10);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started");
  ROS_INFO("Waiting for polygon points. Need at least 3 to form a polygon...");
  std::vector<std::string> saved_areas {};
  std::vector<nav_msgs::Path> all_paths;
  while (ros::ok())
  {
    if (got_initial_pose)
    {
      goal.property.header.stamp = ros::Time::now();
      goal.property.header.frame_id = "map";
      polygon_pub.publish(goal.property);

      goal.robot_position.pose.orientation.w = 1.0;
      goal.robot_position = initial_pose;
      // goal.robot_position.header = goal.property.header;
      // goal.robot_position.pose.position.x = 1.0;
      // goal.robot_position.pose.position.y = 1.0;

      start_pub.publish(goal.robot_position);

      client.sendGoal(goal);
      ROS_INFO_STREAM("Sending goal");

      // wait for the action to return
      bool finished_before_timeout = client.waitForResult(ros::Duration(300.0));

      if (!finished_before_timeout)
      {
        ROS_INFO("Action did not finish before the time out.");
        continue;
      }

      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
      boustrophedon_msgs::PlanMowingPathResultConstPtr result = client.getResult();
      std::cout << "Result with : " << result->plan.points.size() << std::endl;

      nav_msgs::Path path;
      
      convertStripingPlanToPath(result->plan, path);

      for (int i=0; i < result->plan.points.size()-1; i++)
      {
        nav_msgs::Path partial_path {};
        partial_path.header.stamp = path.header.stamp;
        partial_path.header.frame_id = path.header.frame_id;
        partial_path.poses.push_back(path.poses[i]);
        partial_path.poses.push_back(path.poses[i+1]);
        path_pub.publish(partial_path);
        ros::Duration(0.2).sleep();
      }


      std::cout << "Enter a name for the specified area trajectory. Names you have already used are : \n \" " << std::endl;
      for (auto name : saved_areas){
        std::cout << name << std::endl;
      }
      std::cout << " \" " << std::endl;

      std::string area_name {}; 
      std::cout << "New name is : " ; 
      std::cin >> area_name ;
      saved_areas.push_back(area_name);
      std::vector<double> path_x {};
      std::vector<double> path_y {};
      std::vector<double> path_yaw {};
      std::vector<double> path_back_ind {};

      for (int i=0; i < result->plan.points.size()-1; i++)
      {
        path_x.push_back(path.poses[i].pose.position.x);
        path_y.push_back(path.poses[i].pose.position.y);
        path_yaw.push_back(0);
        path_back_ind.push_back(0);
      }
      n.setParam(area_name + "/x_path", path_x);
      n.setParam(area_name + "/y_path", path_y);
      n.setParam(area_name + "/yaw_path", path_yaw);
      n.setParam(area_name + "/back_ind", path_back_ind);

      all_paths.push_back(path);
      for (auto p : all_paths){
        
        path_pub.publish(p);
        ros::Duration(0.2).sleep();
      }
      std::cout << "Path saved successfully, ready to accept next area." << std::endl;

      got_initial_pose = false;
      goal = {};
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z)
{
  // get orientation from heading vector
  const tf2::Vector3 heading_vector(x, y, z);
  const tf2::Vector3 origin(1, 0, 0);

  const auto w = (origin.length() * heading_vector.length()) + tf2::tf2Dot(origin, heading_vector);
  const tf2::Vector3 a = tf2::tf2Cross(origin, heading_vector);
  tf2::Quaternion q(a.x(), a.y(), a.z(), w);
  q.normalize();

  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w()))
  {
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
  }

  return tf2::toMsg(q);
}
