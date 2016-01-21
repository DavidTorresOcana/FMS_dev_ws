#include "mex.h"
#include "matrix.h"
#include <windows.h>

// For configuring DMO properties
#include <wmcodecdsp.h>

// For discovering microphone array device
#include <MMDeviceApi.h>
#include <devicetopology.h>

// For functions and definitions used to create output file
#include <dmo.h> // Mo*MediaType
#include <uuids.h> // FORMAT_WaveFormatEx and such
#include <mfapi.h> // FCC

// For string input,output and manipulation
#include <tchar.h>
#include <strsafe.h>
#include <conio.h>

// For CLSID_CMSRKinectAudio GUID

#include "MSRKinectAudio.h"
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Get pointer to Kinect handles
    unsigned __int64 *MXadress;
    if(nrhs==0) {
        mexErrMsgTxt("Give Pointer to Audio Kinect as input");
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
    
	// WAVEFORMATEX wfxOut= ((WAVEFORMATEX**)MXadress[0])[0];
	BYTE* pbOutputBuffer = ((BYTE**)MXadress[1])[0]; 
	ISoundSourceLocalizer* pSC = ((ISoundSourceLocalizer**)MXadress[2])[0];
	//CStaticMediaBuffer outputBuffer = ((CStaticMediaBuffer*)MXadress[3])[0];
	//bytessecond = (int)MXadress[4];

    delete [] pbOutputBuffer;
    pSC->Release();
}



