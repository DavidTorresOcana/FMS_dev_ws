#include "mex.h"
#include "matrix.h"
#include <Windows.h>
#include "MSR_NuiApi.h"

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    HRESULT hr;
	
   	// Get pointer to Kinect handles
	unsigned __int64 *MXadress;
    if(nrhs==0) {
        mexErrMsgTxt("Give Pointer to Kinect as input");
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
    int depthwidth=(int)MXadress[7];
	int depthheight=(int)MXadress[8];

	// Initialize Output Skeleton Array
    int Jdimsc[2];
    Jdimsc[0]=200; Jdimsc[1]=6;
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxDOUBLE_CLASS, mxREAL);
    double *Pos;
    Pos = mxGetPr(plhs[0]);
	
    NUI_SKELETON_FRAME SkeletonFrame;


	// Wait for a Skeleton_Frame to arrive	
	hr = NuiSkeletonGetNextFrame( 200, &SkeletonFrame );
    if( FAILED( hr ) )
	{ 
		printf("Failed to get Frame\r\n");
	} 
	else
	{
		// Check if there is a Skeleton found
        bool NoSkeletonFound = true;
        for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
            if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
            {
                NoSkeletonFound = false;
            }
        }
        if( NoSkeletonFound )  { return; }
      
        // Smooth the skeleton data
        NuiTransformSmooth(&SkeletonFrame,NULL);

		// Copy Skeleton points to output array
        int r=0;
        for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
            if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
            {
                NUI_SKELETON_DATA * pSkel = &SkeletonFrame.SkeletonData[i];
                int j;
                float fx=0,fy=0;
                for (j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) 
                { 
                    Pos[j+r]=i+1;
                    Pos[j+r+Jdimsc[0]]=pSkel->SkeletonPositions[j].x;
                    Pos[j+r+Jdimsc[0]*2]=pSkel->SkeletonPositions[j].y;
                    Pos[j+r+Jdimsc[0]*3]=pSkel->SkeletonPositions[j].z;    
                    NuiTransformSkeletonToDepthImageF( pSkel->SkeletonPositions[j], &fx, &fy ); 
                    Pos[j+r+Jdimsc[0]*4] = (double) ( fx * depthwidth + 0.5f ); 
                    Pos[j+r+Jdimsc[0]*5] = (double) ( fy * depthheight + 0.5f ); 

                }
                r+=NUI_SKELETON_POSITION_COUNT;
            }
        }
    }
}
