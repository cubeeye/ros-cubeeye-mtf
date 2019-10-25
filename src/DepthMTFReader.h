#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "./include/MTF_API.h"
#define IMAGE_NUM 2
#define DATA_AMPLITUDE 0
#define DATA_DEPTH 1
#define MTF_WIDTH 320
#define MTF_HEIGHT 240

#define PUB_DEPTH "/depth/mtf/depth_raw"
#define PUB_AMPLITUDE "/depth/mtf/amplitude_raw"
#define PUB_PCL "/depth/mtf/points"

class DepthMTFReader
{
  private:
    int m_nDeviceCount;
    mtfHandle m_DevHnd;
    mtfDeviceInfo m_DevInfo[MAX_DEVICE];
    mtfFrameInfo m_FrameInfo[MAX_DEVICE];

    float* m_pDepthData;
    float* m_pAmplitudeData;


  public:
    bool debug;
    bool setMTFParam;
    int m_nWidth;
    int m_nHeight;
    int m_nMinDepth, m_nMaxDepth;
    int m_nInteg;
    int m_nAmplitude;
    int m_nScattering;
    int m_nDelay;
    int m_nOffset;
    bool mLoopOk;

    unsigned short *mCameraBuf[IMAGE_NUM];
    mtfCameraSpacePoint *m_p3DXYZCoordinates;

    DepthMTFReader(ros::Publisher &pub_depth_raw, ros::Publisher &pub_amplitude_raw, ros::Publisher &pub_pcl_raw);
    bool connect();
    bool close();

    int GetDepthData();
    int setAmplitudeThreshold();
    int setScatteringCheckThreshold();
    int setPhaseOffset();

    ros::Publisher &m_PubDepthRaw;
    ros::Publisher &m_PubAmplitudeRaw;
    ros::Publisher &m_PubPCLRaw;

    sensor_msgs::ImagePtr m_msgImgPtrDepth;
    sensor_msgs::ImagePtr m_msgImgPtrAmplitude;
    sensor_msgs::PointCloud2 m_msgPCL2;
    sensor_msgs::PointCloud2Ptr m_msgPCL2ptr;
};
