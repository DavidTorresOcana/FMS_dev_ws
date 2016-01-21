#include "mex.h"
#include "matrix.h"
#include <Windows.h> 
#include "MSR_NuiApi.h"

HANDLE  m_pDepthStreamHandle;
HANDLE  m_pVideoStreamHandle;
INuiInstance*  m_pNuiInstance(NULL);
    
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
	// Get pointer to Kinect handles
    unsigned __int64 *MXadress;
    if(nrhs==0) {
       mexErrMsgTxt("Close failed: Give Pointer to Kinect as input"); 
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
    
	// Close the Video and Depth stream
	m_pNuiInstance=((INuiInstance**)MXadress[0])[0]; 
    m_pVideoStreamHandle=((HANDLE*)MXadress[1])[0]; 
	m_pDepthStreamHandle=((HANDLE*)MXadress[2])[0]; 
	
     // Disable skeleton tracking
    m_pNuiInstance->NuiSkeletonTrackingDisable();

    // Shutdown the Kinect Process
    m_pNuiInstance->NuiShutdown();
       
       
    CloseHandle(m_pVideoStreamHandle);
    CloseHandle(m_pDepthStreamHandle);
    
}
