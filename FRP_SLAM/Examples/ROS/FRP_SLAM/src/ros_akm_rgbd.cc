/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include"../../../include/System.h"
#include <image_transport/image_transport.h>
#include "opencv2/core/eigen.hpp"
#include <std_srvs/Empty.h>
using namespace std;
ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;
ros::Publisher odom_pub;
cv::Mat Camera_Pose;
tf::Transform orb_slam;
tf::TransformBroadcaster * orb_slam_broadcaster;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);
image_transport::Publisher tracking_img_pub;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub;
ros::Time current_time, last_time;
double lastx=0,lasty=0,lastz=0,lastthx=0,lastthy=0,lastthz=0;
unsigned int a =0,b=0;
bool mbActivateLocalizationMode_call = false;
class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_camerapath,pub_odom;
    nav_msgs::Path  camerapath;

    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
        pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 100);
        pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 100);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};
bool deactivateLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res)
{
    mbActivateLocalizationMode_call = true;
    ROS_INFO("开启正常SLAM模式！");
    return true;
}
sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "/odom_orb";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf::Vector3 point_translation(P3Dw.z(), -P3Dw.x(), -P3Dw.y());

            float data_array[num_channels] = {
                    point_translation.x(),
                    point_translation.y(),
                    point_translation.z()
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

void publish_tracking_img(cv::Mat image, ros::Time msg_time)
{
    std_msgs::Header header;

    header.stamp = msg_time;

    header.frame_id = "/odom_orb";

    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    tracking_img_pub.publish(rendered_image_msg);
}

void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

    tracked_mappoints_pub.publish(cloud);
}

void publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

    all_mappoints_pub.publish(cloud);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    bool mode1 = false;
    bool mode2 = false;
    std::string dense(argv[3]);
    std::string purelocation(argv[4]);

    if(dense == "true")
        mode1 = true;
    else
        mode1 = false;
    if(purelocation == "true")
        mode2 = true;
    else
        mode2 = false;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],mode1, mode2, ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ///
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    /**
     * 消息丢失： 如果消息发布的速率高于消息处理的速率，队列将开始积压消息。如果队列大小不足以容纳这些消息，一些消息可能会被丢弃，从而导致数据丢失。

延迟： 队列的存在可以引入一定的消息传递延迟，因为消息需要等待在队列中等待处理。如果队列很大，延迟可能会变得更加明显。

内存消耗： 较大的消息队列会占用更多的内存，因此需要更多的系统资源。如果队列过大，可能会导致节点消耗大量内存。

稳定性： 较小的队列可能导致节点在处理负载高的情况下出现崩溃或崩溃。较大的队列可以提高节点的稳定性，因为它们可以容纳更多的消息。

因此，在选择消息队列大小时，需要综合考虑以上因素，并根据具体的应用需求和性能测试来确定合适的大小。不同的应用可能需要不同大小的队列以达到最佳性能和稳定性。
     */
    ///
    // pub track_image
    image_transport::ImageTransport image_transport(nh);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",100);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 100);
    tracking_img_pub = image_transport.advertise( "/ORB_SLAM3/tracking_image", 1);
    tracked_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "/ORB_SLAM3/tracked_points", 1);
    all_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "ORB_SLAM3/all_points", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("vision_odom", 50);
    ros::ServiceServer service = nh.advertiseService("deactivate_localization", deactivateLocalizationCallback);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("FrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // Save camera trajectory
    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    geometry_msgs::Pose sensor_pose;
    Sophus::SE3f sophus_Tcw;
    sophus_Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat Tcw = cv::Mat(4, 4, CV_32F);
    cv::eigen2cv(sophus_Tcw.matrix(), Tcw);
    ///
    orb_slam_broadcaster = new tf::TransformBroadcaster;
    publish_tracking_img(mpSLAM->GetCurrentFrame(), cv_ptrRGB->header.stamp);
    publish_tracked_points(mpSLAM->GetTrackedMapPoints(), cv_ptrRGB->header.stamp);
    publish_all_points(mpSLAM->GetAllMapPoints(), cv_ptrRGB->header.stamp);
    if (!Tcw.empty())
    {
        cv::Mat Twc =Tcw.inv();
        cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);
        cv::Mat tWC=  Twc.rowRange(0,3).col(3);
        cv::Mat twc(3,1,CV_32F);
        twc = tWC;
        Eigen::Matrix<double,3,3> eigMat ;
        eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
        Eigen::Quaterniond q(eigMat);

        Pose_quat[0] = q.x(); Pose_quat[1] = q.y();
        Pose_quat[2] = q.z(); Pose_quat[3] = q.w();

        Pose_trans[0] = twc.at<float>(0);
        Pose_trans[1] = twc.at<float>(1);
        //Pose_trans[1] = 0;
        Pose_trans[2] = twc.at<float>(2);

        sensor_pose.position.x = twc.at<float>(0);
        sensor_pose.position.y = twc.at<float>(1);
        sensor_pose.position.z = twc.at<float>(2);
        sensor_pose.orientation.x = q.x();
        sensor_pose.orientation.y = q.y();
        sensor_pose.orientation.z = q.z();
        sensor_pose.orientation.w = q.w();

        orb_slam.setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
        orb_slam.setRotation(tf::Quaternion(q.z(), -q.x(), -q.y(), q.w()));
        orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/odom_orb", "/base"));


        Cam_Pose.header.stamp =ros::Time::now();
        //Cam_Pose.header.seq = msgRGB->header.seq;
        Cam_Pose.header.frame_id = "/odom_orb";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);

        Cam_odom.header.stamp = ros::Time::now();
        Cam_odom.header.frame_id = "/odom_orb";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_odom.pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_odom.pose.pose.orientation);
        Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01};
        CamPose_Pub.publish(Cam_Pose);
        Camodom_Pub.publish(Cam_odom);
        ///
        // 创建 nav_msgs::Odometry 消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "/odom_orb"; // 设置坐标系
        odom_msg.pose.pose.position = Cam_odom.pose.pose.position;
        odom_msg.pose.pose.orientation = Cam_odom.pose.pose.orientation;

        // Set the velocity
        odom_msg.child_frame_id = "/base";
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        double vx = (Cam_odom.pose.pose.position.x - lastx)/dt;
        double vy = (Cam_odom.pose.pose.position.y - lasty)/dt;
        double vz = (Cam_odom.pose.pose.position.z - lastz)/dt;
        double vthx = (Cam_odom.pose.pose.orientation.x - lastthx)/dt;
        double vthy = (Cam_odom.pose.pose.orientation.y - lastthy)/dt;
        double vthz = (Cam_odom.pose.pose.orientation.z - lastthz)/dt;

// 设置线速度和角速度信息
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = vz;
        odom_msg.twist.twist.angular.x = vthx;
        odom_msg.twist.twist.angular.y = vthy;
        odom_msg.twist.twist.angular.z = vthz;
/**
 * 在回调函数内部，我们首先获取当前时间戳 current_time，然后计算时间差 dt，
 * 最后将 last_time 更新为 current_time，以便在下一次回调时使用。
 */
// 发布 nav_msgs::Odometry 消息
        // Publish the message
        odom_pub.publish(odom_msg);
        last_time = current_time;
        lastx = Cam_odom.pose.pose.position.x;
        lasty = Cam_odom.pose.pose.position.y;
        lastz = Cam_odom.pose.pose.position.z;
        lastthx = Cam_odom.pose.pose.orientation.x;
        lastthy = Cam_odom.pose.pose.orientation.y;
        lastthz = Cam_odom.pose.pose.orientation.z;

        ///
        std_msgs::Header header ;
        header.stamp =msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id="/odom_orb";
        camerapath.header =header;
        camerapath.poses.push_back(Cam_Pose);
        pub_camerapath.publish(camerapath);  //相机轨迹

    }

}




