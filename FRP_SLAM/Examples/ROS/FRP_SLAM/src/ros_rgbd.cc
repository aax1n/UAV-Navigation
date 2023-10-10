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

cv::Mat Camera_Pose;
tf::Transform orb_slam;
tf::TransformBroadcaster * orb_slam_broadcaster;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);
image_transport::Publisher tracking_img_pub;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub;
bool mbActivateLocalizationMode_call = false;
class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_camerapath;
    nav_msgs::Path  camerapath;

    ImageGrabber(FRP_SLAM::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
        pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 100);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    FRP_SLAM::System* mpSLAM;
};
bool deactivateLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res)
{
    mbActivateLocalizationMode_call = true;
    ROS_INFO("开启正常SLAM模式！");
    return true;
}
sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<FRP_SLAM::MapPoint*> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "/odom";
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

    header.frame_id = "/odom";

    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    tracking_img_pub.publish(rendered_image_msg);
}

void publish_tracked_points(std::vector<FRP_SLAM::MapPoint*> tracked_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

    tracked_mappoints_pub.publish(cloud);
}

void publish_all_points(std::vector<FRP_SLAM::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

    all_mappoints_pub.publish(cloud);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
//    bool mode1 = false;
//    bool mode2 = false;
//    std::string dense(argv[3]);
//    std::string purelocation(argv[4]);
//
//    if(dense == "true")
//        mode1 = true;
//    else
//        mode1 = false;
//    if(purelocation == "true")
//        mode2 = true;
//    else
//        mode2 = false;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    FRP_SLAM::System SLAM(argv[1],argv[2], FRP_SLAM::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    // pub track_image
    image_transport::ImageTransport image_transport(nh);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",100);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 100);
    tracking_img_pub = image_transport.advertise( "/FRP_SLAM/tracking_image", 1);
    tracked_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "/FRP_SLAM/tracked_points", 1);
    all_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "FRP_SLAM/all_points", 1);
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
        //Pose_trans[1] = twc.at<float>(1);
        Pose_trans[1] = 0;
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
        orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/odom", "/orb_cam_link"));

        Cam_Pose.header.stamp =ros::Time::now();
        //Cam_Pose.header.seq = msgRGB->header.seq;
        Cam_Pose.header.frame_id = "/odom";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);

        Cam_odom.header.stamp = ros::Time::now();
        Cam_odom.header.frame_id = "/odom";
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
        std_msgs::Header header ;
        header.stamp =msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id="/odom";
        camerapath.header =header;
        camerapath.poses.push_back(Cam_Pose);
        pub_camerapath.publish(camerapath);  //相机轨迹

    }

}




