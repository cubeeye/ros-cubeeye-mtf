#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/Image.h>
#include "./include/MTF_API.h"
#include "DepthMTFReader.h"
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <depth_mtf/depth_mtfConfig.h>

DepthMTFReader* mr;

void gracefulShutdown(int sigNum)
{
    mr->mLoopOk = false;
}

void callbackConfig(depth_mtf_node::depth_mtfConfig &config, uint32_t level)
{
    if(mr==NULL)
        return;

    switch (level)
    {
    case 1:
        mr->m_nAmplitude = config.Amplitude;
        mr->SetAmplitudeThreshold();
        break;
    case 2:
        mr->m_nScattering = config.Scattering;
        mr->SetScatteringCheckThreshold();
        break;

    default:
        break;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_mtf_node");
    ros::NodeHandle nh;

    ros::Publisher pub_depth_raw = nh.advertise<sensor_msgs::Image>(PUB_DEPTH, 1);
    ros::Publisher pub_amplitude_raw = nh.advertise<sensor_msgs::Image>(PUB_AMPLITUDE, 1);
    ros::Publisher pub_pcl_raw = nh.advertise<sensor_msgs::PointCloud2>(PUB_PCL, 1);

    dynamic_reconfigure::Server<depth_mtf_node::depth_mtfConfig> server;
    dynamic_reconfigure::Server<depth_mtf_node::depth_mtfConfig>::CallbackType f;

    f = boost::bind(&callbackConfig, _1, _2);
    server.setCallback(f);

    mr = new DepthMTFReader(pub_depth_raw,pub_amplitude_raw,pub_pcl_raw);


    nh.getParam("/depth_camera/setMTFParam", mr->setMTFParam);    
    nh.getParam("/depth_camera/check_thresh", mr->m_nAmplitude);
    nh.getParam("/depth_camera/scatter_thresh", mr->m_nScattering);
    nh.getParam("/depth_camera/min_depth", mr->m_nMinDepth);
    nh.getParam("/depth_camera/max_depth", mr->m_nMaxDepth);


    ROS_INFO("setMTFParam : %d\n", mr->setMTFParam);

    if (!mr->connect())
    {
        ROS_ERROR("Depth Camera Connection Failed!...");
        std::exit(1);
    }

    //to be in inverse proportion to the fps delay(nDelay) parameter.
    ros::Rate loop_rate(10);

    signal(SIGINT, gracefulShutdown);

    ROS_INFO("Depth Camera Start\n");

    while (mr->mLoopOk)
    {
        if (mr->mCameraBuf[DATA_DEPTH] != NULL)
        {
            //make publish
            if(mr->GetDepthData() == ERROR_NO)
            {
                pub_amplitude_raw.publish(mr->m_msgImgPtrAmplitude);
                pub_depth_raw.publish(mr->m_msgImgPtrDepth);
                pub_pcl_raw.publish(mr->m_msgPCL2ptr);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    mr->close();
    ROS_INFO("Depth Camera Stop\n");
    return 0;
}
