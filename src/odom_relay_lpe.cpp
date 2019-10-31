
// ROS
#include "ros/ros.h"

#include <Eigen/Dense>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>

ros::Publisher pose_pub;
ros::Publisher odom_pub;
geometry_msgs::Point COM;
double min_dt_mavros;
double last_pub_mavros;

geometry_msgs::Point set_point(const double &x, const double &y, const double &z) {
  geometry_msgs::Point v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

geometry_msgs::Vector3 set_vector3(const double &x, const double &y, const double &z) {
  geometry_msgs::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

geometry_msgs::Point add_point(const geometry_msgs::Point &v1,
                               const geometry_msgs::Point &v2) {
  return set_point(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

geometry_msgs::Point convert_to_inertial_frame(
    const geometry_msgs::Quaternion &quat,
    const geometry_msgs::Point &pt) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Quaternion q_body(pt.x, pt.y, pt.z, 0.0);
  tf::Quaternion q_pt_world = q*q_body*q.inverse();
  geometry_msgs::Point pt_world = set_point(q_pt_world.x(), q_pt_world.y(), q_pt_world.z());
  return pt_world;
}

// Here we assume that the velocity is computed in body frame!
geometry_msgs::Vector3 velocity_center_of_mass(
    const geometry_msgs::Vector3 &v_camera,
    const geometry_msgs::Point &cam_2_com,
    const geometry_msgs::Vector3 &ang_vel) {
  // v_com = v_camera - cross(ang_vel, com_2_cam)

  // Assume center of mass and camera frames are aligned
  Eigen::Vector3d v_cam(v_camera.x, v_camera.y, v_camera.z);
  Eigen::Vector3d com_2_cam(-cam_2_com.x, -cam_2_com.y, -cam_2_com.z);
  Eigen::Vector3d omega(ang_vel.x, ang_vel.y, ang_vel.z);
  Eigen::Vector3d omega_cross_r = omega.cross(com_2_cam);
  Eigen::Vector3d v_com = v_cam - omega_cross_r;

  return set_vector3(v_com[0], v_com[1], v_com[2]);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "local_origin";
  pose.pose.orientation = msg->pose.pose.orientation;

  // Publish pose with added COM
  const geometry_msgs::Point cam_pos = msg->pose.pose.position;
  const geometry_msgs::Quaternion q_cam = msg->pose.pose.orientation;
  const geometry_msgs::Point cam2com_world = convert_to_inertial_frame(q_cam, COM);
  pose.pose.position = add_point(cam_pos, cam2com_world);

  // Odometry of COM
  nav_msgs::Odometry odom = *msg;
  odom.pose.pose = pose.pose;
  odom.twist.twist.linear = msg->twist.twist.linear;
    // velocity_center_of_mass(msg->twist.twist.linear, COM, msg->twist.twist.angular);
  odom.header.frame_id = "local_origin";
  odom.child_frame_id = "com_frame";

  odom_pub.publish(odom);

  // Publish in mavros if enough time has passed
  double dt = msg->header.stamp.toSec() - last_pub_mavros;
  if (dt > min_dt_mavros) {
    pose_pub.publish(pose);
    last_pub_mavros = msg->header.stamp.toSec();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

  // Center of mass position w.r.t. the camera
  double COM_x, COM_y, COM_z;
  n.getParam("COM_x", COM_x);
  n.getParam("COM_y", COM_y);
  n.getParam("COM_z", COM_z);
  COM = set_point(COM_x, COM_y, COM_z);

  // Get frequency to publish into mavros
  double mavros_freq;
  n.getParam("mavros_freq", mavros_freq);
  min_dt_mavros = 1.0/mavros_freq;
  last_pub_mavros = 0.0;

  std::string ns;
  n.getParam("namespace", ns);

  std::string in_odom_topic, out_pose_topic, out_odom_com_topic;
  if (ns.length() > 0) {
    in_odom_topic = "/" + ns + "/t265/odom/sample";
    out_pose_topic = "/" + ns + "/mavros/vision_pose/pose";
    out_odom_com_topic = "/" + ns + "/t265/odom/center_of_mass";
  } else {
    in_odom_topic = "/t265/odom/sample";
    out_pose_topic = "/mavros/vision_pose/pose";
    out_odom_com_topic = "/t265/odom/center_of_mass";
  }

  ros::Subscriber sub = n.subscribe(in_odom_topic, 5, odomCallback);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>(out_pose_topic, 5);
  odom_pub = n.advertise<nav_msgs::Odometry>(out_odom_com_topic, 5);
  
  ros::spin();

  return 0;
} 