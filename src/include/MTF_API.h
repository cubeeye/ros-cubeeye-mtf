/*****************************************************************************
*
* Copyright (C) 2013 meerecompany Ltd.

* MTF_API.h : MTF_API.so's default header file.
* Meere ToF SDK(Linux Shared Version)
* Copyright(c) Meerecompany, All rights reserved.
* Update: 2018/01/24 Rev - ver. 2.4 [ For new model ]
*
*****************************************************************************/

#ifndef MTF_API_H
#define MTF_API_H



#define MAX_DEVICE 10
typedef int mtfHandle;

//Error List
typedef enum mtfError
{
    ERROR_NO        = 1,
    ERROR_FAIL      = 0,
    ERROR_HANDLE    = -1,
    ERROR_OPEN      = -2,
    ERROR_TIME_OUT  = -3,
    ERROR_PARAM     = -4,
    ERROR_READ_DATA = -5,
}mtfError;

//Filtering Type
typedef enum mtfFilter
{
    FILTER_NO           = 0,
    FILTER_SMOOTHING    = 1,
}mtfFilter;

//Undistortion Mode Type
typedef enum _mtfUndistortionMode
{
    DISTORTION_NO			= 0,//Undistortion Off
    DISTORTION_MODE1		= 1,//Undistortion On + Interpolation On
    DISTORTION_MODE2		= 2,//Undistortion On + Interpolation Off
    DISTORTION_MODE3		= 3,//Undistortion On + Interpolation On & Edge Remove On

} _mtfUndistortionMode;


//Device Information
typedef struct mtfDeviceInfo
{
    mtfHandle mtfHnd;           //Device Handle
    char szDevPath[256];        //Device Path
    char szVendor[256];         //Vendor Name
    char szName[256];           //Name
    char szSerialNum[256];      //Serial Number
    unsigned short nVendorId;   //Vendor ID
    unsigned short nProductId;  //Product ID
    unsigned short nDeviceType; //Device Type(0:Only Depth, 1:Depth+RGB)
}mtfDeviceInfo;

//Device Infomation
typedef unsigned long long ULONGLONG;
typedef struct _mtfFrameInfo
{
    int nWidth;					//Image Width
    int nHeight;				//Image Height

    long nFrameIndex;			//Frmae Index
    ULONGLONG nTimeStamp;		//Frame Timestamp

} mtfFrameInfo;

//Lens Parameter Infomation(support to new model - later 2016.07~)
typedef struct _mtfDistortionParam
{
    float fK1;	//X.XXX
    float fK2;	//X.XXX
    float fK3;	//X.XXX
    float fP1;	//X.XX
    float fP2;	//X.XX
    float fSkew;//X.XXX

} mtfDistortionParam;

typedef struct _mtfIntrinsicParam
{
    float fFx;	//XXX.X
    float fFy;	//XXX.X
    float fCx;	//XXX.X
    float fCy;	//XXX.X

} mtfIntrinsicParam;

typedef struct _mtfExtrinsicParam
{
    float fR11;
    float fR12;
    float fR13;
    float fR21;
    float fR22;
    float fR23;
    float fR31;
    float fR32;
    float fR33;
    float fTx;
    float fTy;
    float fTz;

} mtfExtrinsicParam;

//3D XYZ Coordinates
typedef struct _mtfCameraSpacePoint
{
    float fX;					//X Coordinates(unit: mm)
    float fY;					//Y Coordinates(unit: mm)
    float fZ;					//Z Coordinates(unit: mm)

} mtfCameraSpacePoint;

//X,Y Pixel, Depth
typedef struct _mtfDepthSpacePoint
{
    float fPointX;				//X Pixel Point (unit: pixel pos)
    float fPointY;				//Y Pixel Point (unit: pixel pos)
    float fDepth;				//Depth (unit: mm)

} mtfDepthSpacePoint;

#ifdef __cplusplus
extern "C" {
#endif

/*
*	@description	get the connected device info. list <mtfDeviceInfo[0] ~ mtfDeviceInfo[9]>
*	@param			*hnd - device handle list
                    *nDevCount - device count
*	@return			success(1) or false(0)
*/
int mtfGetDeviceList(mtfDeviceInfo *pDevInfo, int *nDevCount);


/*
*	@description	device Open
*	@param			hnd - device handle number
*	@return			success(1) or false(0)
*/
int mtfDeviceOpen(mtfHandle hnd, int nRGBCam_NO);


/*
*	@description	check device open
*	@param			hnd - device handle number
*	@return			open(1) or close(0)
*/
int mtfDeviceIsOpen(mtfHandle hnd);


/*
*	@description	device close
*	@param			hnd - device handle number
*	@return			success(1) or false(0)
*/
int mtfDeviceClose(mtfHandle hnd);


/*
*	@description	set Integration time (all set function is applied during reading thread run(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
                    nIntegrationTime - Intgrationtime value(1000 ~ 5000 [unit:us])
*	@return			success(1) or false(0)
*/
int mtfSetIntegrationTime(mtfHandle hnd, int nIntegrationTime);


/*
*	@description	get Integration time
*	@param			hnd - device handle number
*	@return			current Intgrationtime value
*/
int  mtfGetIntegrationTime(mtfHandle hnd);


/*
*	@description	set Module Frequency (all set function is applied during reading thread run.(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
                    nModuleFrequency - Module Frequency value(LDC Type: 10 [unit:Mhz], MDC Type: 20 [unit:Mhz]) - don't change value
*	@return			success(1) or false(0)
*/
int mtfSetModuleFrequency(mtfHandle hnd, int nModuleFrequency);


/*
*	@description	get Module Frequency
*	@param			hnd - device handle number
*	@return			current Module Frequency
*/
int  mtfGetModuleFrequency(mtfHandle hnd);


/*
*	@description	set distance offset(all set function is applid during reading thread run.(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
                    nOffset - Offset value(0 ~ MaxDistance [unit:mm])
*	@return			success(1) or false(0)
*/
int mtfSetOffset(mtfHandle hnd, int nOffset);


/*
*	@description	get distance offset
*	@param			hnd - device handle number
*	@return			current offset
*/
int  mtfGetOffset(mtfHandle hnd);


/*
*	@description	set depth signal check threshold
*	@param			hnd - device handle number
                    nThreshold - nThreshold value(0 ~ 255)
*	@return			success(1) or false(0)
*/
int mtfSetCheckThreshold(mtfHandle hnd, int nThreshold);


/*
*	@description	get depth signal check threshold (all set function is applied during reading thread run.(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
*	@return			current depth signal check threshold
*/
int  mtfGetCheckThreshold(mtfHandle hnd);

/*
*	@description	set scattering threshold function
*	@param			hnd - device handle number
                    nThreshold - scattering check threshold value(0 ~ 255)
*	@return			success(1) or false(0)
*/
int mtfSetScatterThreshold(mtfHandle hnd, int nThreshold);

/*
*	@date			2016. 04. 27.
*	@description	get Scattering theshold
*	@param			hnd - device handle number
*	@return			current Scattering theshold
*/
int mtfGetScatterThreshold(mtfHandle hnd);


/*
*	@description	read buffer init (call this function once before 'mtfReadFromDevice' function thread run)
*	@param			hnd - device handle number
                    wRecvData - [0]:Amplitude data, [1]:Depth data
*	@return			void
*/
int mtfReadBufInit(mtfHandle hnd);


/*
*	@description	grap start (call this function once after reading thread run.(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
*	@return			success(1) or false(0)
*/
int mtfGrabStart(mtfHandle hnd);


/*
*	@description	grap stop (call this function once before reading thread run.(thread continuously call 'ReadFromDevice��function)
*	@param			hnd - device handle number
*	@return			success(1) or false(0)
*/
int mtfGrabStop(mtfHandle hnd);


/*
*	@description	get depth data from device
*	@param			hnd - device handle number
                    wRecvData - [0]:Amplitude data(320 x 240), [1]:Depth data(320 x 240)
*	@return			void
*/
int mtfReadFromDevice(mtfHandle hnd, unsigned short** wRecvData, mtfFrameInfo *stFrameInfo);


/*
*	@description	get rgb data from device
*	@param			hnd - device handle number
                    RGBData - RGB 888 data(640 x 480 x 3)
*	@return			void
*/
int mtfReadFromDevice_RGB888(mtfHandle hnd, unsigned char* nRGBData, mtfFrameInfo *stFrameInfo);


/*
*	@description	set depth range(out of depth value is zero)
*	@param			hnd - device handle number
                    nMinDepth - min depth
                    nMaxDepth - max dpeth
*	@return			success(1) or false(0)
*/
int mtfSetDepthRange(mtfHandle hnd, int nMinDepth, int nMaxDepth);


/*
*	@description	get depth range
*	@param			hnd - device handle number
                    nMinDepth - min depth
                    nMaxDepth - max dpeth
*	@return			success(1) or false(0)
*/
int mtfGetDepthRange(mtfHandle hnd, int *nMinDepth, int *nMaxDepth);

/*
*	@description	set Flip Horizontal for frame
*	@param			hnd - device handle number
                    bEnable - true to enable, false to disable Flip Horizontal
*	@return			success(1) or false(0)
*/
int mtfSetFlipHorizontal(mtfHandle hnd, int nEnable);


/*
*	@description	get Flip Horizontal status
*	@param			hnd - device handle number
*	@return			true if 'Flip Horizontal' is currently enabled, false otherwise.
*/
int mtfGetFlipHorizontal(mtfHandle hnd);


/*
*	@description	set Flip Vertical for frame
*	@param			hnd - device handle number
                    bEnable - true to enable, false to disable Flip Vertical
*	@return			success(1) or false(0)
*/
int mtfSetFlipVertical(mtfHandle hnd, int nEnable);


/*
*	@description	get Flip Vertical status
*	@param			hnd - device handle number
*	@return			true if 'Flip Vertical' is currently enabled, false otherwise.
*/
int mtfGetFlipVertical(mtfHandle hnd);

/*
*	@description	get device sensor temperature
*	@param			hnd - device handle number
*	@return			temperature
*/
double mtfGetTemperature(mtfHandle hnd);

/*
*	@date			2015. 11. 26.
*	@description	set Multi Sync mode status
*	@param			hnd - device handle number
                    bEnable - true to enable
*	@return			success(1) or false(0)
*/
int mtfSetMultiSyncMode(mtfHandle hnd, int nEnable);


/*
*	@date			2015. 11. 26.
*	@description	get Multi Sync mode status
*	@param			hnd - device handle number
*	@return			success(1) or false(0)
*/
int mtfGetMultiSyncMode(mtfHandle hnd);


/*
*	@date			2015. 10. 22.
*	@description	set FPS Delay
*	@param			hnd - device handle number
                    nDelay - Delay time
*	@return			success(1) or false(0)
*/
int mtfSetFPSDelay(mtfHandle hnd, int nDelay);


/*
*	@date			2015. 10. 22.
*	@description	get FPS Delay
*	@param			hnd - device handle number
*	@return			current FPS Delay
*/
int mtfGetFPSDelay(mtfHandle hnd);


/*
*	@description	set enable/disable remove edge filter status
*	@param			hnd - device handle number

*	@return			success(1) or false(0)
*/
int mtfSetRemoveEdge(mtfHandle hnd, int nEnable);


/*
*	@description	get remove edge filter status
*	@param			hnd - device handle number

*	@return			enable(1) or disable(0)
*/
int mtfGetRemoveEdge(mtfHandle hnd);


/*
*	@description	depth frame to Camera coordinates
*	@param			wDepth - Raw depth frame data,
                    pCameraSpacePoints: X,Y,Z Camera coordinates(unit:mm)
                    IntrinsicParam - Lens Intrinsic parameters
*	@return			void
*/
void mtfDepthFrameToCameraSpace(unsigned short* pDepth, mtfCameraSpacePoint *pCameraSpacePoint, int nWidth, int nHeight);

/*
*	@description	Camera space point to depth point
*	@param			pCameraSpacePoint - X,Y,Z Camera coordinates(unit:m)
                    pDepthPoint - PointX, PointY (unit: pixel), Depth (unit: mm)
                    IntrinsicParam - Lens Intrinsic parameters

*	@return			void
*/
void mtfCameraSpacePointToDepth(mtfCameraSpacePoint pCameraSpacePoint, mtfDepthSpacePoint *pDepthPoint, mtfIntrinsicParam IntrinsicParam);


/*
*	@description	depth point to Camera space point
                    pDepthPoint - X,Y (unit: pixel), Depth (unit: mm)
                    pCameraSpacePoints - X,Y,Z Camera coordinates(unit:m)
                    IntrinsicParam - Lens Intrinsic parameters

*	@return			void
*/
void mtfDepthPointToCameraSpace(mtfDepthSpacePoint pDepthPoint, mtfCameraSpacePoint *pCameraSpacePoint, mtfIntrinsicParam IntrinsicParam);

/*
*	@description	coordinates conversion with extrinsic parameter
*	@param			pCameraPoint1 - Input X,Y,Z coordinates(unit:mm)
                    pCameraPoint2 - Output X,Y,Z coordinates(unit:mm)
                    ExtrinsicParam - Extrinsic Parameter

*	@return			void
*/
void mtfXYZ3DConversion(mtfCameraSpacePoint pCameraPoint1, mtfCameraSpacePoint *pCameraPoint2, mtfExtrinsicParam ExtrinsicParam);

/*
*	@description	depth frame coordinates to RGB Color frame coordinates
*	@param			ExtrinsicParam - Extrinsic Paramerter

*	@return			Reverse Extrinsic Parameter
*/
mtfExtrinsicParam mtfGetReverse(mtfExtrinsicParam ExtrinsicParam);


/*
*	@description	set lens undistortion  mode
*	@param			hnd - device handle number
                    nDistortionMode - Undistortion Off: 0, Undistortion On + Interpolation On & Edge Remove On:1, Undistortion On + Interpolation On:2, Undistortion On + Interpolation Off:3

*	@return			success(1) or false(0)
*/
int mtfSetUndistortion(mtfHandle hnd, int nDistortionMode);



/*
*	@description	get lens distortion correction enable/disable status
*	@param			hnd - device handle number

*	@return			Enable(1) or Disable(0)
*/
int mtfGetUndistortion(mtfHandle hnd);


/*
*	@description	get lens distortion correction enable/disable status
*	@param			hnd - device handle number
                    nPixelX - Distort pixel X Pos
                    nPixelY - Distort pixel Y Pos

*	@return			UnDistort pixel x pos, UnDistort pixel y pos
*/
void mtfGetUndistortPixel(mtfHandle hnd, int *nPixelX, int *nPixelY);


/*
*	@description	convert orthogonal z data from radial depth data
*	@param			hnd - device handle number
                    nEnable - true to enable, false to disable

*	@return			success(1) or false(0)
*/
int mtfSetOrthogonal(mtfHandle hnd, int nEnable);


/*
*	@description	get enable/disable status
*	@param			hnd - device handle number

*	@return			Enable(1) or Disable(0)
*/
int mtfGetOrthogonal(mtfHandle hnd);


/*
*	@description	set lens parameter
*	@param			hnd - device handle number
                    stLensParam - lens parameter structure

*	@return			success(1) or false(0)
*/
int mtfSetLensParameter(mtfHandle hnd, mtfIntrinsicParam stIntrinsicParam, mtfDistortionParam stDistortionParam);


/*
*	@description	get lens parameter
*	@param			hnd - device handle number
                    stLensParam - lens parameter structure

*	@return			success(1) or false(0)
*/
int mtfGetLensParameter(mtfHandle hnd, mtfIntrinsicParam *stIntrinsicParam, mtfDistortionParam *stDistortionParam);


/*
*	@description	set lens parameter
*	@param			hnd - device handle number
                    stIntrinsicParam - lens Intrinsic parameter structure
                    stDistortionParam - lens distortion parameter structure

*	@return			success(1) or false(0)
*/
int mtfSetRGBLensParameter(mtfHandle hnd, mtfIntrinsicParam stIntrinsicParam, mtfDistortionParam stDistortionParam);

/*
*	@description	get lens parameter
*	@param			hnd - device handle number
                    stIntrinsicParam - lens Intrinsic parameter structure
                    stDistortionParam - lens distortion parameter structure

*	@return			success(1) or false(0)
*/
int mtfGetRGBLensParameter(mtfHandle hnd, mtfIntrinsicParam *stIntrinsicParam, mtfDistortionParam *stDistortionParam);

/*
*	@description	set extrinsic parameter
*	@param			hnd - device handle number
                    stExtrinsicParam - extrinsic parameter structure

*	@return			success(1) or false(0)
*/
int mtfSetExtrinsicParameter(mtfHandle hnd, mtfExtrinsicParam stExtrinsicParam);

/*
*	@description	get extrinsic parameter
*	@param			hnd - device handle number
                    stExtrinsicParam - extrinsic parameter structure

*	@return			success(1) or false(0)
*/
int mtfGetExtrinsicParameter(mtfHandle hnd, mtfExtrinsicParam *stExtrinsicParam);

/*
*	@description	Amplitude Strength Check Filter for record file playback
*	@param			wAmplitudeData - Amplitude Data Buf
                                        wDepthData     - Depth Data Buf
                                        nThreshold - Check Threshold

*	@return			void
*/
void mtfAmplitudeCheckFilter(unsigned short* wAmplitudeData, unsigned short* wDepthData, unsigned char nThreshold, int nWidth, int nHeight);

/*
*	@description	Scattering Check Filter for record file playback
*	@param			wAmplitudeData - Amplitude Data Buf
                                        wDepthData     - Depth Data Buf
                                        nThreshold - Check Threshold

*	@return			void
*/
void mtfScatteringCheckFilter(unsigned short* wAmplitudeData, unsigned short* wDepthData, unsigned char nThreshold, int nWidth, int nHeight);

/*
*	@description	Min ~ Max Depth Check Filter for record file playback
*	@param			wDepthData - Depth Data Buf
                                        nMinDepth  - Check Min Depth Value
                                        nMaxDepth  - Check Max Depth Value

*	@return			void
*/
void mtfMinMaxCheckFilter(unsigned short* wDepthData, unsigned short nMinDepth,  unsigned short nMaxDepth, int nWidth, int nHeight);

/*
*	@description	set enable/disable multi cam mode for wired multi camera's
*	@param			hnd - device handle number
                    bMultimode -  true to enable, false to disable multi cam mode
                    nTotalCameraCount - total slave cam count(1~9)
                    nCameraID - Slave Cam ID(0~9, master cam:0, slave cam:1~9)

    @derail         setting example for 2 camera - mtfSetMultiCamMode(0, true, 1, 0) //master camera
                                                 - mtfSetMultiCamMode(1, true, 1, 1) //master camera

*	@return			success(1) or false(0)
*/
int mtfSetMultiCamMode(mtfHandle hnd, int nMultimode, int nTotalCameraCount, int nCameraID);

#ifdef __cplusplus
}
#endif


#endif // MTF_API_H
