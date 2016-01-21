#include "mex.h"
#include "matrix.h"
#include <Windows.h>
#include "MSR_NuiApi.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    HANDLE  m_pVideoStreamHandle;
	HRESULT hr;
    
  // Get pointer to Kinect handles
    unsigned __int64 *MXadress;
    if(nrhs==0) {
        mexErrMsgTxt("Give Pointer to Kinect as input");
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
    m_pVideoStreamHandle=((HANDLE*)MXadress[1])[0];
	int imagewidth=(int)MXadress[5];
	int imageheight=(int)MXadress[6];
	
	// Initialize Output image
	int Jdimsc[3]={imageheight, imagewidth, 3};
    plhs[0] = mxCreateNumericArray(3, Jdimsc, mxUINT8_CLASS, mxREAL);
    byte *Iout;
    Iout = (byte*)mxGetData(plhs[0]);

    	
    // Wait for a Video_Image_Frame to arrive
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
	hr = NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 200, &pImageFrame );
    if( FAILED( hr ) )
	{ 
		printf("Failed to get Frame\r\n");
	} 
	else
	{
		// Convert the Video_Image_Frame to an output image
        INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
        NUI_LOCKED_RECT LockedRect;
        pTexture->LockRect( 0, &LockedRect, NULL, 0 );
        if( LockedRect.Pitch != 0 ) {
            BYTE * pBuffer = (BYTE*) LockedRect.pBits;
            int j=0;
            int index;
            for(int x=0; x<Jdimsc[0]; x++) {
                for(int y=0; y<Jdimsc[1]; y++) {
                    for(int c=0; c<Jdimsc[2]; c++) {
                        index=x+y*Jdimsc[0]+(2-c)*(Jdimsc[0]*Jdimsc[1]);
                        Iout[index]=pBuffer[j]; j++;
                    }
                    j++;
                }
            }
        }
        else {
            printf("Video Buffer Length Error\r\n");
        }
        NuiImageStreamReleaseFrame( m_pVideoStreamHandle, pImageFrame );
    }
}
