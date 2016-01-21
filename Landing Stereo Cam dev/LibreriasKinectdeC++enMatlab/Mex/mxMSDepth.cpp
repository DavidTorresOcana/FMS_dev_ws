#include "mex.h"
#include "matrix.h"
#include <Windows.h>
#include "MSR_NuiApi.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    HANDLE  m_pDepthStreamHandle;
    HRESULT hr;
   
	// Get pointer to Kinect handles
    unsigned __int64 *MXadress;
    if(nrhs==0) {
        mexErrMsgTxt("Give Pointer to Kinect as input");
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
    m_pDepthStreamHandle=((HANDLE*)MXadress[2])[0];
    int depthwidth=(int)MXadress[7];
	int depthheight=(int)MXadress[8];

    // Initialize Output image
    int Jdimsc[2]={depthheight, depthwidth};
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT16_CLASS, mxREAL);
	unsigned short *Iout;
    Iout = (unsigned short*)mxGetData(plhs[0]);
 		
	// Wait for a Depth_Image_Frame to arrive
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    hr = NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 200, &pImageFrame );
    if( FAILED( hr ) )
	{ 
		printf("Failed to get Frame\r\n");
	} 
	else
	{
		// Convert the Depth_Image_Frame to an output image
        INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
        NUI_LOCKED_RECT LockedRect;
        pTexture->LockRect( 0, &LockedRect, NULL, 0 );
        if( LockedRect.Pitch != 0 ) {
            USHORT* pBuffer =  (USHORT*) LockedRect.pBits;
            int j=0;
            int index;
            for(int x=0; x<Jdimsc[0]; x++) {
                for(int y=0; y<Jdimsc[1]; y++) {
                    index=x+y*Jdimsc[0];
                    Iout[index]=pBuffer[j]; j++;
                }
            }
        }
        else {
            printf("Depth Buffer Length Error\r\n");
        }
        NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
    }
}
