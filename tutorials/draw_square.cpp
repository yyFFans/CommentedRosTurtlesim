#include <boost/bind.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;

enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;

#define PI 3.141592

void poseCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.1 && fabsf(g_pose->y - g_goal.y) < 0.1 && fabsf(g_pose->theta - g_goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

void printGoal()
{
  ROS_INFO("New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

void stopForward(ros::Publisher twist_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    // wrap g_goal.theta to [-pi, pi)
    if (g_goal.theta >= PI) g_goal.theta -= 2 * PI;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}

void stopTurn(ros::Publisher twist_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


void forward(ros::Publisher twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 1.0, 0.0);
  }
}

void turn(ros::Publisher twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 0.0, 0.4);
  }
}

void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  if (!g_pose)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(twist_pub);
  }
  else if (g_state == STOP_FORWARD)
  {
    stopForward(twist_pub);
  }
  else if (g_state == TURN)
  {
    turn(twist_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(twist_pub);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_square");
  ros::NodeHandle nh;

  // turtlesim中更新时 turtle1发布pose话题 从而触发回调poseCallback 保存位姿
  ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);

  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");

  // 定时器超时，触发twist_pub 发布turtle1/cmd_vel话题，turtlesim中turtle1回调velocityCallback函数进行位姿更新
  // turtlesim 窗口更新 触发pose发布，回调poseCallback 更新位姿
  // 如此往复
  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));

  std_srvs::Empty empty;
  reset.call(empty);

  ros::spin();
}
