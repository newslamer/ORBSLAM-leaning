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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

//添加库
#include <geometry_msgs/PoseStamped.h> //modify
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include"../../../include/Converter.h"
#include <nav_msgs/Path.h>

using namespace std;

// 声明类（图像收集类）
class ImageGrabber
{
public:
    //构造函数的申明与定义（申明一个指针）——C++函数后面跟：表示赋值
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    /*相当于
    ImageGrabber（ORB_SLAM2::System* pSLAM）
    {
        mpSLAM = pSLAM
    }//初始化操作
    */

    //消息回调函数
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,
                    const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;  //初始化SLAM系统,mpSLAM对应pSLAM

};

ros::Publisher pose_pub;
nav_msgs::Path rgbd_path;
ros::Publisher rgbd_path_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    //argc：remapping参数的个数
    //argv：remapping参数的列表 
    //name：节点名，必须是一个基本名称，不能包含命名空间
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,false);



    //三个输入
    //词袋库位置
    //配置文件位置
    //是否开启用户界面

    ImageGrabber igb(&SLAM);
    //建立图像采集类


    ros::NodeHandle nh;
    //句柄(Handle)这个概念可以理解为一个“把手”，你握住了门把手，就可以很容易把整扇门拉开，而不必关心门是什么样子。
    //NodeHandle就是对节点资源的描述，有了它你就可以操作这个节点了，比如为程序提供服务、监听某个topic上的消息、访问和修改param等等

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 5);
    //创建ROS节点（subscriber)，订阅由rgbd传感器采集的rgb图像和深度图像信息
    //接受话题

    //异源消息融合，将深度图与色彩图同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);//sync_pol(10)表示queue size=10

    //message_filters用于对齐多种传感信息的时间戳，对齐时间戳有两种方式
    //一种是时间戳完全对齐 ：ExactTime Policy 
    //另一种是时间戳相近：ApproximateTime Policy

    //boost::bind(f, 1, 2)可以产生一个无参函数对象，返回f(1, 2)；
    //对类方法来说，直接boost::bind(&类名::方法名，类实例指针，参数1，参数2）
    //"_1"。这个叫做站位符，他代表这个位置有个参数，但现在还不知道参数是什么。_1代表参数列表中的第一个位置上的参数
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    //发布位置信息
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 3);//
    rgbd_path_pub = nh.advertise<nav_msgs::Path>("ORB_SLAM/path",1);//


    ros::spin();
    //ros::spin()，这可以理解为一个动作，打开订阅者的嘴。这样订阅者们可以开始接受话题，最重要的是进入回调函数！

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

// 消息回调函数
// 定义消息回调函数（输入为两个消息常量）
// 将消息转换为opencv下的矩阵类型
// 传入跟踪线程
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,
                            const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    //cv_bridge用于ROS图像和OpenCV图像的转换（转变为常量）
    cv_bridge::CvImageConstPtr cv_ptrRGB;

    //try是错误捕获，根据错误类型报错
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

    //开始进入跟踪线程
    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    //开始进入跟踪线程,并实时输出位置
    cv::Mat Tcw;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id ="path";

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    //3行3列再转置
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    //3行第4列
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
    tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    new_transform.setRotation(quaternion);
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);
    
    rgbd_path.header.frame_id="path";
    rgbd_path.header.stamp=ros::Time::now();
    rgbd_path.poses.push_back(pose);
    rgbd_path_pub.publish(rgbd_path);

}


