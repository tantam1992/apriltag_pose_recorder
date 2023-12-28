#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <unordered_set>

using namespace std;

ros::Subscriber ar_sub_;

class Localizer
{
public:
  Localizer(ros::NodeHandle &nh)
  {
    // Set up subscribers
    ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Localizer::number_callback, this);

    // Set the map frame ID
    nh.param<std::string>("map_frame_id", mapFrameId, "map");

    // Set the output file name
    nh.param<std::string>("output_file", outputFile, "/home/u/tko_ws/src/apriltag_pose_recorder/data/tag_poses.txt");
  }

  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
  {
    for (const auto &detection : msg->detections)
    {
      if (!detection.id.empty())
      {
        int tagId = detection.id[0];

        if (recordedTags.count(tagId) == 0)
        {
          // Transform pose to map frame
          geometry_msgs::PoseStamped cameraPose;
          cameraPose.header = detection.pose.header;
          cameraPose.pose = detection.pose.pose.pose;

          try
          {
            tfListener.waitForTransform(mapFrameId, cameraPose.header.frame_id, cameraPose.header.stamp, ros::Duration(1.0));
            tfListener.transformPose(mapFrameId, cameraPose, cameraPose);
          }
          catch (const tf::TransformException &ex)
          {
            ROS_ERROR("Transform failed: %s", ex.what());
            return;
          }

          float getPosX = cameraPose.pose.position.x;
          float getPosY = cameraPose.pose.position.y;
          float getPosZ = cameraPose.pose.position.z;

          float getOriX = cameraPose.pose.orientation.x;
          float getOriY = cameraPose.pose.orientation.y;
          float getOriZ = cameraPose.pose.orientation.z;
          float getOriW = cameraPose.pose.orientation.w;

          cout << "Point position:" << endl;
          cout << "Tag ID: " << tagId << " posX: " << getPosX << " posY: " << getPosY << " posZ: " << getPosZ << " oriX: " << getOriX << " oriY: " << getOriY << " oriZ: " << getOriZ << " oriW: " << getOriW <<endl;

          savePoseToFile(tagId, getPosX, getPosY, getPosZ, getOriX, getOriY, getOriZ, getOriW);

          // Mark tag as recorded
          recordedTags.insert(tagId);
        }
      }
      else
      {
        ROS_WARN("Empty tag ID detected.");
        return;
      }
    }
  }

  void savePoseToFile(int tagId, float getPosX, float getPosY, float getPosZ, float getOriX, float getOriY, float getOriZ, float getOriW)
  {
    std::ofstream outputFileStream(outputFile, std::ios::app);

    if (outputFileStream.is_open())
    {
      // Write the tag ID and pose to the file
      outputFileStream << "Tag ID: " << tagId << " Position (x, y, z): " << getPosX << ", " << getPosY << ", " << getPosZ << " Orientation (x, y, z, w): " << getOriX << ", " << getOriY << ", " << getOriZ << ", " << getOriW << "\n";
    }
    else
    {
      ROS_ERROR("Unable to open output file: %s", outputFile.c_str());
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber ar_sub_;
  tf::TransformListener tfListener;
  std::string mapFrameId;
  std::string outputFile;
  std::unordered_set<int> recordedTags;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_pose_recorder");
  ros::NodeHandle node_obj;
  Localizer localizer(node_obj);
  ROS_INFO("Apriltag Pose Recorder Start!");
  ros::spin();
  return 0;
}
