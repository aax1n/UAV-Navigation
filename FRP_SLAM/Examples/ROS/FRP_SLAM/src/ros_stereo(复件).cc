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
Eigen::Vector3f Pose_trans;
Eigen::Quaternionf Pose_quat;
image_transport::Publisher tracking_img_pub;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub;
bool mbActivateLocalizationMode_call = false;
class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_camerapath;
    nav_msgs::Path  camerapath;
    ImageGrabber(FRP_SLAM::System* pSLAM):mpSLAM(pSLAM){
        pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 100);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    FRP_SLAM::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
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
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun FRP_SLAM Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    FRP_SLAM::System SLAM(argv[1],argv[2],FRP_SLAM::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    image_transport::ImageTransport image_transport(nh);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/infra2/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",100);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 100);
    tracking_img_pub = image_transport.advertise( "/FRP_SLAM/tracking_image", 1);
    tracked_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "/FRP_SLAM/tracked_points", 1);
    all_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>( "FRP_SLAM/all_points", 1);
    ros::ServiceServer service = nh.advertiseService("deactivate_localization", deactivateLocalizationCallback);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else{
        geometry_msgs::Pose sensor_pose;
        Sophus::SE3f sophus_Tcw;
        sophus_Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());

        // 直接使用Sophus类型进行位姿操作
        Sophus::SE3f Twc = sophus_Tcw.inverse(); // Twc = Tcw的逆矩阵

        Pose_trans = Twc.translation(); // 使用Eigen类型存储平移部分
        Pose_quat = Twc.unit_quaternion(); // 使用Eigen类型存储四元数

        sensor_pose.position.x = Pose_trans.x();
        sensor_pose.position.y = Pose_trans.y();
        sensor_pose.position.z = Pose_trans.z();
        sensor_pose.orientation.x = Pose_quat.x();
        sensor_pose.orientation.y = Pose_quat.y();
        sensor_pose.orientation.z = Pose_quat.z();
        sensor_pose.orientation.w = Pose_quat.w();

        orb_slam.setOrigin(tf::Vector3(Pose_trans.z(), -Pose_trans.x(), -Pose_trans.y()));
        orb_slam.setRotation(tf::Quaternion(Pose_quat.x(), -Pose_quat.y(), -Pose_quat.z(), Pose_quat.w()));
        orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/odom", "/orb_cam_link"));

        Cam_Pose.header.stamp = ros::Time::now();
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
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/odom";
        camerapath.header = header;
        camerapath.poses.push_back(Cam_Pose);
        pub_camerapath.publish(camerapath); //相机轨迹
    }


}


