#include "mex.h"
#include "matrix.h"
#include <Windows.h> 
#include "MSR_NuiApi.h"

// Handles to Depth and Video stream must be here (public/global) 
HANDLE  m_pDepthStreamHandle;
HANDLE  m_pVideoStreamHandle;
INuiInstance*  m_pNuiInstance(NULL);
        
	
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// Output the Point to the Kinect Object
	int Jdimsc[2]={1,9};
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT64_CLASS, mxREAL);
	unsigned __int64 *MXadress;
	MXadress = (unsigned __int64*)mxGetData(plhs[0]);
	
	// Check input for image resolution;
	int imageres=2;
	int depthres=1;
	if(nrhs==1) {
	   if( mxGetNumberOfElements(prhs[0])!=2) { mexErrMsgTxt("Resolution Parameters length must be 2"); }
	   if(!mxIsDouble(prhs[0])){ mexErrMsgTxt("Resolution Parameters must be double"); }
	   double* idres = mxGetPr(prhs[0]);
	   imageres=(int)idres[0];
	   depthres=(int)idres[1];
	}
    if(nrhs>1)
    {
        mexErrMsgTxt("Too many input arguments");
    }
    HRESULT hr;

    hr = MSR_NuiCreateInstanceByIndex((int)0, &m_pNuiInstance);
    if( FAILED( hr ) )
    {
		mexErrMsgTxt("Kinect Initialization Failed");
    }
	else
	{
		printf("Initialized Kinect\r\n");
	}
    
	// Initialize Kinect
    hr = m_pNuiInstance->NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
    if( FAILED( hr ) )
    {
		mexErrMsgTxt("Kinect Initialization Failed");
    }
	else
	{
		printf("Initialized Kinect\r\n");
	}

	// Initialize Video
    m_pVideoStreamHandle = NULL;
	int imagewidth,imageheight;
	NUI_IMAGE_RESOLUTION res;
	switch(imageres)
	{
        case 0:
			imagewidth=80; imageheight=60; res=NUI_IMAGE_RESOLUTION_80x60; break;
        case 1:
			imagewidth=320; imageheight=240; res=NUI_IMAGE_RESOLUTION_320x240; break;
        case 2:
			imagewidth=640; imageheight=480; res=NUI_IMAGE_RESOLUTION_640x480; break;
        case 3:
			imagewidth=1280; imageheight=1024;  res=NUI_IMAGE_RESOLUTION_1280x1024; break;
		default :
			imagewidth=640; imageheight=480;  res=NUI_IMAGE_RESOLUTION_640x480; break;
	}
	
    hr = m_pNuiInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, res,0,2, NULL, &m_pVideoStreamHandle );
    if( FAILED( hr ) )
    {
		mexErrMsgTxt("Video Sensor Initialization Failed\r\n");
    }
	else
	{
		printf("Initialized Video Sensor\r\n");
	}

	// Initialize Depth Video
    m_pDepthStreamHandle = NULL;	
	int depthwidth,depthheight;
	switch(depthres)
	{
        case 0:
			depthwidth=80; depthheight=60; res=NUI_IMAGE_RESOLUTION_80x60; break;
        case 1:
			depthwidth=320; depthheight=240; res=NUI_IMAGE_RESOLUTION_320x240; break;
        case 2:
			depthwidth=640; depthheight=480; res=NUI_IMAGE_RESOLUTION_640x480; break;
        case 3:
			depthwidth=1280; depthheight=1024;  res=NUI_IMAGE_RESOLUTION_1280x1024; break;
		default :
			depthwidth=320; depthheight=240;  res=NUI_IMAGE_RESOLUTION_640x480; break;
	}
	
    hr = m_pNuiInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,res,0,2, NULL,&m_pDepthStreamHandle );
    if( FAILED( hr ) )
    {
		mexErrMsgTxt("Depth Sensor Initialization Failed\r\n");
    }
	else
	{
		printf("Initialized Depth Sensor\r\n");
	}
	
	// Initialize Skeleton Tracking
    
    hr =  m_pNuiInstance->NuiSkeletonTrackingEnable( NULL, 0 );
    if( FAILED( hr ) )
    {
		printf("Skeleton Tracking Initialization Failed\r\n");
    }
	else
	{
		printf("Initialized Skeleton Tracking\r\n");
	}
    
    
	// Pointer Adress of Kinect Handles
    MXadress[0] = ( unsigned __int64)&m_pNuiInstance;
	MXadress[1] = ( unsigned __int64)&m_pVideoStreamHandle;
	MXadress[2] = ( unsigned __int64)&m_pDepthStreamHandle;
	// MXadress[3]
	// MXadress[4]
	MXadress[5]=imagewidth;
	MXadress[6]=imageheight;
	MXadress[7]=depthwidth;
	MXadress[8]=depthheight;
}
