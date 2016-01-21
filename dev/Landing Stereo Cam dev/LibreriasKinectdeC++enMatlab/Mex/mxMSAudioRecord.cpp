// Note Parts of this code are from Microsoft Kinect Example, Non-commercial Use only
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

class CStaticMediaBuffer : public IMediaBuffer {
public:
    CStaticMediaBuffer() {}
    CStaticMediaBuffer(BYTE *pData, ULONG ulSize, ULONG ulData) :
        m_pData(pData), m_ulSize(ulSize), m_ulData(ulData), m_cRef(1) {}
        STDMETHODIMP_(ULONG) AddRef() { return 2; }
        STDMETHODIMP_(ULONG) Release() { return 1; }
        STDMETHODIMP QueryInterface(REFIID riid, void **ppv) {
            if (riid == IID_IUnknown) { 
				AddRef(); *ppv = (IUnknown*)this; return NOERROR; 
			}
            else if (riid == IID_IMediaBuffer) {
                AddRef(); *ppv = (IMediaBuffer*)this; return NOERROR;
            }
            else
                return E_NOINTERFACE;
        }
        STDMETHODIMP SetLength(DWORD ulLength) {m_ulData = ulLength; return NOERROR;}
        STDMETHODIMP GetMaxLength(DWORD *pcbMaxLength) {*pcbMaxLength = m_ulSize; return NOERROR;}
        STDMETHODIMP GetBufferAndLength(BYTE **ppBuffer, DWORD *pcbLength) {
            if (ppBuffer) *ppBuffer = m_pData;
            if (pcbLength) *pcbLength = m_ulData;
            return NOERROR;
        }
        void Init(BYTE *pData, ULONG ulSize, ULONG ulData) {
            m_pData = pData; m_ulSize = ulSize; m_ulData = ulData;
        }
protected:
    BYTE *m_pData; ULONG m_ulSize; ULONG m_ulData; ULONG m_cRef;
};


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    HRESULT hr;
    
    // Get pointer to Kinect handles
    unsigned __int64 *MXadress;
    if(nrhs==0) {
        mexErrMsgTxt("Give Pointer to Audio Kinect as input");
    }
    MXadress = (unsigned __int64*)mxGetData(prhs[0]);
  
	WAVEFORMATEX wfxOut= ((WAVEFORMATEX*)MXadress[0])[0];
	BYTE* pbOutputBuffer = ((BYTE**)MXadress[1])[0]; 
	ISoundSourceLocalizer* pSC = ((ISoundSourceLocalizer**)MXadress[2])[0];
	CStaticMediaBuffer outputBuffer = ((CStaticMediaBuffer*)MXadress[3])[0];
	IMediaObject* pDMO = ((IMediaObject**)MXadress[4])[0];
                
	int bytessecond = (int)MXadress[5];

	int  cTtlToGo = 0;
    DWORD cOutputBufLen = 0;
    ULONG cbProduced = 0;
    DWORD dwStatus;
    
    DMO_OUTPUT_DATA_BUFFER OutputBufferStruct = {0};
    OutputBufferStruct.pBuffer = &outputBuffer;
	

    // allocate output buffer
    cOutputBufLen = wfxOut.nSamplesPerSec * wfxOut.nBlockAlign;
    pbOutputBuffer = new BYTE[cOutputBufLen];
    if (pbOutputBuffer==NULL) { mexErrMsgTxt("out of memory.\n"); }
    
    int  iDuration = 20;   // seconds
    if(nrhs>1)
    {
        double *iDurationd = (double*)mxGetData(prhs[1]);
        iDuration=(int)iDurationd[0];
    }
    
            
    // number of frames to record
    cTtlToGo = iDuration * 100;
    
    DWORD written = 0;
    int totalBytes = 0;
    double dBeamAngle, dAngle;
    
    // Make output array
    int Jdimsc[2]={bytessecond*iDuration/2+1000,1};
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxINT16_CLASS, mxREAL);
	byte* AudioOut;
	AudioOut = (byte*)mxGetData(plhs[0]);

    int Adimsc[2]={bytessecond*iDuration+1000,3};
    double *PosOut;
    if(nlhs>1)
    {
        plhs[1] = mxCreateNumericArray(2, Adimsc, mxDOUBLE_CLASS, mxREAL);
        PosOut = (double*)mxGetData(plhs[1]);   
    }
    
    // main loop to get mic output from the DMO
    printf("\nAEC-MicArray is running ...\n");
    while (1)
    {
        Sleep(10); //sleep 10ms
        
        if (cTtlToGo--<=0)
            break;
        do{
            outputBuffer.Init((byte*)pbOutputBuffer, cOutputBufLen, 0);
            OutputBufferStruct.dwStatus = 0;
            hr = pDMO->ProcessOutput(0, 1, &OutputBufferStruct, &dwStatus);
            if (FAILED(hr)){ mexErrMsgTxt("ProcessOutput failed"); }
            if (hr == S_FALSE) {
                cbProduced = 0;
            }
            else
            {
                hr = outputBuffer.GetBufferAndLength(NULL, &cbProduced);
                if (FAILED(hr)){ mexErrMsgTxt("GetBufferAndLength failed"); }
            }
       
            if( (totalBytes+(int)cbProduced) <Jdimsc[0] )
            {
                memcpy(&AudioOut[totalBytes],pbOutputBuffer,cbProduced);
            } 
            else
            {
                printf("Output Array Size to small");
            }
            // Obtain beam angle from ISoundSourceLocalizer afforded by microphone array
            hr = pSC->GetBeam(&dBeamAngle);
            double dConf;
            hr = pSC->GetPosition(&dAngle, &dConf);
            if(SUCCEEDED(hr))
            {
                if(nlhs>1)
                {
                    if(totalBytes<Adimsc[0])
                    {
                        PosOut[totalBytes]=dAngle;
                        PosOut[totalBytes+Adimsc[0]]=dConf;
                        PosOut[totalBytes+Adimsc[0]*2]=dBeamAngle;
                    }
                }
            }
            totalBytes += cbProduced;
            
        } while (OutputBufferStruct.dwStatus & DMO_OUTPUT_DATA_BUFFERF_INCOMPLETE);
    }

}



