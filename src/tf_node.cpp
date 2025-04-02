#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

void markerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int marker_id) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "camera_frame"; // Assuming poses are in camera frame
    transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;

    transform.transform.rotation = msg->pose.orientation;

    br.sendTransform(transform);
}

geometry_msgs::TransformStamped getMarkerRelativePose(tf2_ros::Buffer& tf_buffer, int reference_marker_id, int target_marker_id) {
    try {
        return tf_buffer.lookupTransform(
            "aruco_marker_" + std::to_string(reference_marker_id),
            "aruco_marker_" + std::to_string(target_marker_id),
            ros::Time(0),
            ros::Duration(1.0)
        );
    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF lookup failed: %s", ex.what());
        throw;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_relative_pose");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Subscriber sub0 = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/marker_0", 10, boost::bind(markerPoseCallback, _1, 0));
    ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/marker_1", 10, boost::bind(markerPoseCallback, _1, 1));
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/marker_2", 10, boost::bind(markerPoseCallback, _1, 2));

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        try {
            geometry_msgs::TransformStamped transform_1_to_0 = getMarkerRelativePose(tf_buffer, 0, 1);
            ROS_INFO("Marker 1 pose relative to Marker 0:");
            ROS_INFO("Position: (%f, %f, %f)",
                     transform_1_to_0.transform.translation.x,
                     transform_1_to_0.transform.translation.y,
                     transform_1_to_0.transform.translation.z);
            ROS_INFO("Orientation: (%f, %f, %f, %f)",
                     transform_1_to_0.transform.rotation.x,
                     transform_1_to_0.transform.rotation.y,
                     transform_1_to_0.transform.rotation.z,
                     transform_1_to_0.transform.rotation.w);

            geometry_msgs::TransformStamped transform_2_to_0 = getMarkerRelativePose(tf_buffer, 0, 2);
            ROS_INFO("Marker 2 pose relative to Marker 0:");
            ROS_INFO("Position: (%f, %f, %f)",
                     transform_2_to_0.transform.translation.x,
                     transform_2_to_0.transform.translation.y,
                     transform_2_to_0.transform.translation.z);
            ROS_INFO("Orientation: (%f, %f, %f, %f)",
                     transform_2_to_0.transform.rotation.x,
                     transform_2_to_0.transform.rotation.y,
                     transform_2_to_0.transform.rotation.z,
                     transform_2_to_0.transform.rotation.w);
        } catch (const tf2::TransformException& ex) {
            ROS_WARN("Failed to get relative pose: %s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
