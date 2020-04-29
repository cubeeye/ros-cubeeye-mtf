#include <std_msgs/String.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "MTF_API.h"
#include "DepthMTFReader.h"
#include <limits>

DepthMTFReader::DepthMTFReader(ros::Publisher &pub_depth_raw, ros::Publisher &pub_amplitude_raw, ros::Publisher &pub_pcl_raw)
    : m_PubDepthRaw(pub_depth_raw)
    , m_PubAmplitudeRaw(pub_amplitude_raw)
    , m_PubPCLRaw(pub_pcl_raw)
{
    m_nWidth = MTF_WIDTH;
    m_nHeight = MTF_HEIGHT;
    m_nDeviceCount = 0;
    mLoopOk = true;

    mtfGetDeviceList(m_DevInfo, &m_nDeviceCount);
    m_DevHnd = m_DevInfo[0].mtfHnd;

    for (int i = 0; i < IMAGE_NUM; i++)
    {
        mCameraBuf[i] = NULL;
        mCameraBuf[i] = (unsigned short *)malloc(sizeof(unsigned short) * m_nWidth * m_nHeight);
    }
    m_p3DXYZCoordinates = NULL;
    m_p3DXYZCoordinates = new mtfCameraSpacePoint[m_nWidth*m_nHeight];

    //generate Depth
    m_msgImgPtrDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrDepth->header.frame_id = "distance";
    m_msgImgPtrDepth->width = m_nWidth;
    m_msgImgPtrDepth->height = m_nHeight;
    m_msgImgPtrDepth->is_bigendian = false;
    m_msgImgPtrDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrDepth->step = (uint32_t)(sizeof(float) * m_nWidth);
    m_msgImgPtrDepth->data.resize(sizeof(float) * m_nWidth * m_nHeight);
    m_pDepthData = (float *)&m_msgImgPtrDepth->data[0];

    //generate Amplitude
    m_msgImgPtrAmplitude = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrAmplitude->header.frame_id = "amplitude";
    m_msgImgPtrAmplitude->width = m_nWidth;
    m_msgImgPtrAmplitude->height = m_nHeight;
    m_msgImgPtrAmplitude->is_bigendian = false;
    m_msgImgPtrAmplitude->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrAmplitude->step = (uint32_t)(sizeof(float) * m_nWidth);
    m_msgImgPtrAmplitude->data.resize(sizeof(float) * m_nWidth * m_nHeight);
    m_pAmplitudeData = (float *)&m_msgImgPtrAmplitude->data[0];

    //generate Pointcloud
    m_msgPCL2ptr = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
    m_msgPCL2ptr->header.frame_id = "pcl";
    m_msgPCL2ptr->header.stamp = m_msgImgPtrAmplitude->header.stamp;
    m_msgPCL2ptr->width = m_nWidth;
    m_msgPCL2ptr->height = m_nHeight;
    m_msgPCL2ptr->is_bigendian = false;
    m_msgPCL2ptr->is_dense = false;

    m_msgPCL2ptr->point_step = (uint32_t)(3 * sizeof(float));
    m_msgPCL2ptr->row_step = (uint32_t)(m_msgPCL2ptr->point_step * m_nWidth);
    m_msgPCL2ptr->fields.resize(3);
    m_msgPCL2ptr->fields[0].name = "z";
    m_msgPCL2ptr->fields[0].offset = 0;
    m_msgPCL2ptr->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[0].count = 1;

    m_msgPCL2ptr->fields[1].name = "y";
    m_msgPCL2ptr->fields[1].offset = m_msgPCL2ptr->fields[0].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[1].count = 1;

    m_msgPCL2ptr->fields[2].name = "x";
    m_msgPCL2ptr->fields[2].offset = m_msgPCL2ptr->fields[1].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[2].count = 1;
    m_msgPCL2ptr->data.resize(m_msgPCL2ptr->point_step * m_msgPCL2ptr->width * m_msgPCL2ptr->height);

}

bool DepthMTFReader::connect()
{
    if (mtfDeviceOpen(m_DevHnd, 0) != ERROR_NO)
    {
        return false;
    }
    mtfReadBufInit(m_DevHnd);
    usleep(100);

    mtfGrabStart(m_DevHnd);

    if (setMTFParam)
    {
        //parameters
        m_nAmplitude = mtfGetCheckThreshold(m_DevHnd);        
        ROS_INFO("Amplitude Threshold : %d  ", m_nAmplitude);

        m_nScattering = mtfGetScatterThreshold(m_DevHnd);        
        ROS_INFO("Scattering Threshold : %d", m_nScattering);

        mtfGetDepthRange(m_DevHnd, &m_nMinDepth, &m_nMaxDepth);        
        ROS_INFO("Depth range : %d - %d : unit=mm", m_nMinDepth, m_nMaxDepth);

    }
    return true;
}

bool DepthMTFReader::close()
{
    ROS_INFO("Stop Grab MTF");

    mtfGrabStop(m_DevHnd);
    ROS_INFO("Device Close MTF");
    mtfDeviceClose(m_DevHnd);

    for (int i = 0; i < IMAGE_NUM; i++)
    {
        if (mCameraBuf[i] != NULL)
        {
            free(mCameraBuf[i]);
            mCameraBuf[i] = NULL;
        }
    }

    if(m_p3DXYZCoordinates != NULL)
    {
        delete m_p3DXYZCoordinates;
        m_p3DXYZCoordinates = NULL;
    }
}

int DepthMTFReader::GetDepthData()
{
    int nRet;

    nRet = mtfReadFromDevice(m_DevHnd, (unsigned short **)mCameraBuf, &m_FrameInfo[m_DevHnd]);
    if(nRet != ERROR_NO)
        return nRet;


    mtfDepthFrameToCameraSpace(mCameraBuf[DATA_DEPTH], m_p3DXYZCoordinates, m_nWidth, m_nHeight);

    // following to the REP117, convert unit of depth values to meter instead of millimeter
    // filter out unreliable values by thresholding the amplitude values
    for (int i = 0; i < m_nWidth * m_nHeight; i++)
    {
        unsigned int raw_amp_value = mCameraBuf[DATA_AMPLITUDE][i];
        unsigned int raw_depth_value = mCameraBuf[DATA_DEPTH][i];

        float float_amp_value = raw_amp_value / 1000.0f;
        float float_depth_value = raw_depth_value / 1000.0f;

        m_pAmplitudeData[i] = float_amp_value;
        m_pDepthData[i] = float_depth_value;

        float *pcl_a = (float *)&m_msgPCL2ptr->data[i * m_msgPCL2ptr->point_step];
        float *pcl_b = pcl_a + 1;
        float *pcl_c = pcl_b + 1;

        *pcl_a = m_p3DXYZCoordinates[i].fY;
        *pcl_b = m_p3DXYZCoordinates[i].fX * (-1);
        *pcl_c = m_p3DXYZCoordinates[i].fZ;
    }

    return nRet;
}

int DepthMTFReader::SetAmplitudeThreshold()
{
    int nRet;
    nRet = mtfSetCheckThreshold(m_DevHnd, m_nAmplitude);
    return nRet;
}

int DepthMTFReader::SetScatteringCheckThreshold()
{
    int nRet;
    nRet = mtfSetScatterThreshold(m_DevHnd, m_nScattering);
    return nRet;
}

