#include "opencv2/aruco/dictionary.hpp"
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <map>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <tf/tf.h>

void setQuaternionFromRvec(cv::Vec<double, 3> &rvec, geometry_msgs::Quaternion &q) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  tf::Matrix3x3 tfR(
      R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
  tf::Quaternion tfQ;
  tfR.getRotation(tfQ);
  q.x = tfQ.x();
  q.y = tfQ.y();
  q.z = tfQ.z();
  q.w = tfQ.w();
}

class Camera {
public:
  Camera(ros::NodeHandle &nh, const std::string &topic,
         const std::string &calib_file,
         std::function<void(const sensor_msgs::ImageConstPtr &)> callback)
      : it_(nh) {
    sub_ = it_.subscribe(topic, 1, callback);
    loadCalibration(calib_file);
  }

  const cv::Mat &getCameraMatrix() const { return cameraMatrix_; }
  const cv::Mat &getDistCoeffs() const { return distCoeffs_; }

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  cv::Mat cameraMatrix_, distCoeffs_;

  void loadCalibration(const std::string &calib_file) {
    std::ifstream file(calib_file);
    if (!file.is_open()) {
      ROS_ERROR("Failed to open calibration file: %s", calib_file.c_str());
      return;
    }

    std::string line;
    std::vector<double> cam_matrix_values, dist_coeffs_values;
    bool parsing_camera_matrix = false;
    bool parsing_distortion = false;

    while (std::getline(file, line)) {
      if (line.find("camera matrix") != std::string::npos) {
        parsing_camera_matrix = true;
        continue;
      }
      if (line.find("distortion") != std::string::npos) {
        parsing_camera_matrix = false;
        parsing_distortion = true;
        continue;
      }
      if (parsing_camera_matrix || parsing_distortion) {
        std::istringstream iss(line);
        double val;
        while (iss >> val) {
          if (parsing_camera_matrix) {
            cam_matrix_values.push_back(val);
          } else if (dist_coeffs_values.size() < 5) {
            dist_coeffs_values.push_back(val);
          }
        }
      }
    }

    if (cam_matrix_values.size() == 9 && dist_coeffs_values.size() == 5) {
      cameraMatrix_ = cv::Mat(3, 3, CV_64F, cam_matrix_values.data()).clone();
      distCoeffs_ = cv::Mat(1, dist_coeffs_values.size(), CV_64F,
                            dist_coeffs_values.data())
                        .clone();
      ROS_INFO_STREAM("Loaded Camera Matrix:\n" << cameraMatrix_);
      ROS_INFO_STREAM("Loaded Distortion Coefficients:\n" << distCoeffs_);
    } else {
      ROS_ERROR("Invalid calibration file format.");
    }
  }
};

class ArucoDetector {
public:
  ArucoDetector(ros::NodeHandle &nh, const std::string &calib_file)
      : nh_(nh), it_(nh), objPoints(4, 1, CV_32FC3) {
    cam1_ = std::make_unique<Camera>(nh_, "/one/image_raw", calib_file,
    std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1,
    "Camera 1"));
    //cam2_ =
    //    std::make_unique<Camera>(nh_, "/four/image_raw", "./david.txt",
    //                             std::bind(&ArucoDetector::imageCallback, this,
    //                                       std::placeholders::_1, "Camera 2"));

    objPoints.ptr<cv::Vec3f>(0)[0] =
        cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] =
        cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] =
        cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] =
        cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
  }

  void spin() { ros::spin(); }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::unique_ptr<Camera> cam1_, cam2_;
  std::map<int, ros::Publisher> marker_publishers_;
  cv::Mat objPoints;
  // Length of the marker side in meters
  const float markerLength = 0.15f;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg,
                     const std::string &cameraName) {
    cv::Mat imageCopy;
    try {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners;
      cv::Ptr<cv::aruco::Dictionary> dictionary =
          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
      image.copyTo(imageCopy);

      if (!markerIds.empty()) {
        const cv::Mat &cameraMatrix = (cameraName == "Camera 1" && cam1_)
                                          ? cam1_->getCameraMatrix()
                                          : cam2_->getCameraMatrix();
        const cv::Mat &distCoeffs = (cameraName == "Camera 1" && cam1_)
                                        ? cam1_->getDistCoeffs()
                                        : cam2_->getDistCoeffs();

        std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
        for (size_t i = 0; i < markerIds.size(); ++i) {
          cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs,
                       rvecs.at(i), tvecs.at(i));
        }

        for (size_t i = 0; i < markerIds.size(); ++i) {
          cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i],
                            tvecs[i], markerLength * 1.5f, 2);
          int id = markerIds[i];
          if (marker_publishers_.find(id) == marker_publishers_.end()) {
            marker_publishers_[id] = nh_.advertise<geometry_msgs::PoseStamped>(
                "/aruco/marker_" + std::to_string(id), 10);
          }
          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header.stamp = ros::Time::now();
          pose_msg.pose.position.x = tvecs[i][0];
          pose_msg.pose.position.y = tvecs[i][1];
          pose_msg.pose.position.z = tvecs[i][2];
          geometry_msgs::Quaternion q;
          setQuaternionFromRvec(rvecs[i], q);
          pose_msg.pose.orientation = q;
          marker_publishers_[id].publish(pose_msg);
          printf("PUBLISHED POSE FOR MARKER %d: (%.2f, %.2f, %.2f)\n", id,
                 tvecs[i][0], tvecs[i][1], tvecs[i][2]);
        }
      }

      if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
      } else {
        // ROS_INFO("NO DETECTED MARKERS\n");
      }

      cv::imshow(cameraName, imageCopy);
      cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;
  if (argc < 2) {
    ROS_ERROR("Usage: rosrun <package_name> aruco_detector <calibration_file>");
    return -1;
  }
  ArucoDetector detector(nh, argv[1]);
  detector.spin();
  return 0;
}