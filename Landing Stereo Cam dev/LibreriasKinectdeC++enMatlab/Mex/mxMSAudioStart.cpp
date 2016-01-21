// Note Parts of this code are from Microsoft Kinect Example, Non-commercial use only
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


#define SAFE_ARRAYDELETE(p) {if (p) delete[] (p); (p) = NULL;}
#define SAFE_RELEASE(p) {if (NULL != p) {(p)->Release(); (p) = NULL;}}

   
///////////////////////////////////////////////////////////////////////////////
// GetJackSubtypeForEndpoint
//
// Gets the subtype of the jack that the specified endpoint device is plugged
// into.  E.g. if the endpoint is for an array mic, then we would expect the
// subtype of the jack to be KSNODETYPE_MICROPHONE_ARRAY
//
///////////////////////////////////////////////////////////////////////////////
HRESULT GetJackSubtypeForEndpoint(IMMDevice* pEndpoint, GUID* pgSubtype)
{
    HRESULT hr = S_OK;
    IDeviceTopology*    spEndpointTopology = NULL;
    IConnector*         spPlug = NULL;
    IConnector*         spJack = NULL;
    IPart*            spJackAsPart = NULL;
    
    if (pEndpoint == NULL)
        return E_POINTER;
    
    // Get the Device Topology interface
    hr = pEndpoint->Activate(__uuidof(IDeviceTopology), CLSCTX_INPROC_SERVER, NULL, (void**)&spEndpointTopology);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    hr=spEndpointTopology->GetConnector(0, &spPlug);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    hr=spPlug->GetConnectedTo(&spJack);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    hr=spJack->QueryInterface(__uuidof(IPart), (void**)&spJackAsPart);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
	hr = spJackAsPart->GetSubType(pgSubtype);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    SAFE_RELEASE(spEndpointTopology);
    SAFE_RELEASE(spPlug);
    SAFE_RELEASE(spJack);
    SAFE_RELEASE(spJackAsPart);
    return hr;
}


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



HRESULT GetKinectMicArrayDeviceIndex(int *piDevice)
{
    HRESULT hr = S_OK;
    UINT index, dwCount;
    IMMDeviceEnumerator* spEnumerator;
    IMMDeviceCollection* spEndpoints;
    
    *piDevice = -1;
    
    hr=CoCreateInstance(__uuidof(MMDeviceEnumerator),  NULL, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&spEnumerator);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    hr=spEnumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &spEndpoints);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    hr=spEndpoints->GetCount(&dwCount);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    
    // Iterate over all capture devices until finding one that is a microphone array
    for (index = 0; index < dwCount; index++)
    {
        IMMDevice* spDevice;
        
        hr=spEndpoints->Item(index, &spDevice);
        if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }

		GUID subType = {0};
        hr=GetJackSubtypeForEndpoint(spDevice, &subType);
        if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
        
        if (subType == KSNODETYPE_MICROPHONE_ARRAY)
        {
            *piDevice = index;
            break;
        }
    }
    
    hr = (*piDevice >=0) ? S_OK : E_FAIL;
    
    SAFE_RELEASE(spEnumerator);
    SAFE_RELEASE(spEndpoints);
    return hr;
}

// Public variables
WAVEFORMATEX wfxOut;
BYTE *pbOutputBuffer = NULL;
ISoundSourceLocalizer* pSC = NULL;
CStaticMediaBuffer outputBuffer;
IMediaObject* pDMO = NULL;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
    HRESULT hr = S_OK;
    CoInitialize(NULL);
    int  iMicDevIdx = -1;
    int  iSpkDevIdx = 0;  //Asume default speakers
    
    IPropertyStore* pPS = NULL;
    
    CoCreateInstance(CLSID_CMSRKinectAudio, NULL, CLSCTX_INPROC_SERVER, IID_IMediaObject, (void**)&pDMO);
    pDMO->QueryInterface(IID_IPropertyStore, (void**)&pPS);
    
    PROPVARIANT pvSysMode;
    PropVariantInit(&pvSysMode);
    pvSysMode.vt = VT_I4;
    pvSysMode.lVal = (LONG)(2); // Microphone without cancelation
    pPS->SetValue(MFPKEY_WMAAECMA_SYSTEM_MODE, pvSysMode);
    PropVariantClear(&pvSysMode);
    
    // Tell DMO which capture device to use (we're using whichever device is a microphone array).
    // Default rendering device (speaker) will be used.
    hr = GetKinectMicArrayDeviceIndex(&iMicDevIdx);
    if (FAILED(hr)){ mexErrMsgTxt("Failed to find microphone array device. Make sure microphone array is properly installed."); }
    
    PROPVARIANT pvDeviceId;
    PropVariantInit(&pvDeviceId);
    pvDeviceId.vt = VT_I4;
    //Speaker index is the two high order bytes and the mic index the two low order ones
    pvDeviceId.lVal = (unsigned long)(iSpkDevIdx<<16) | (unsigned long)(0x0000ffff & iMicDevIdx);
    hr=pPS->SetValue(MFPKEY_WMAAECMA_DEVICE_INDEXES, pvDeviceId);
    if (FAILED(hr)){ mexErrMsgTxt("Audio Error"); }
    PropVariantClear(&pvDeviceId);

    //	PCM audio
    //	1 channel
    //	16,000 samples/second
    //	32,000 bytes/second, on average
    //	A block alignment of 2, which means that the DMO processes 2 bytes of data at a time
    //	16 bits/sample
    //	No extra format information
    int samplessecond=16000;
    int wBitsPerSample=16;
    int channels=1;
    int nblockalign=(wBitsPerSample*channels)/8;
    int bytessecond = nblockalign*samplessecond;
    // WAVE_FORMAT_PCM, WAVE_FORMAT_IEEE_FLOAT one or two canals
    // WAVE_FORMAT_EXTENSIBLE	PCM data that has more than two channels or uses a channel mask.
    // WAVE_FORMAT_ADPCM
    // WAVE_FORMAT_XMA2
    // WAVE_FORMAT_WMAUDIO2
    // WAVE_FORMAT_WMAUDIO3
    WORD wFormatTag=WAVE_FORMAT_PCM;
    wfxOut.wFormatTag = wFormatTag;
    wfxOut.nChannels =  channels;
    wfxOut.nSamplesPerSec = samplessecond;
    wfxOut.nAvgBytesPerSec = bytessecond;
    wfxOut.nBlockAlign = nblockalign;
    wfxOut.wBitsPerSample = wBitsPerSample;
    wfxOut.cbSize = 0;
    
    DMO_MEDIA_TYPE mt = {0};
    
    // Set DMO output format
    hr = MoInitMediaType(&mt, sizeof(WAVEFORMATEX));
    if (FAILED(hr)){ mexErrMsgTxt("MoInitMediaType failed"); }
    mt.majortype = MEDIATYPE_Audio;
    mt.subtype = MEDIASUBTYPE_PCM;
    mt.lSampleSize = wBitsPerSample/8;
    mt.bFixedSizeSamples = TRUE;
    mt.bTemporalCompression = FALSE;
    mt.formattype = FORMAT_WaveFormatEx;
    memcpy(mt.pbFormat, &wfxOut, sizeof(WAVEFORMATEX));
    
    hr = pDMO->SetOutputType(0, &mt, 0);
    if (FAILED(hr)){ mexErrMsgTxt("SetOutputType failed"); }
    
    MoFreeMediaType(&mt);
    
    // Allocate streaming resources. This step is optional. If it is not called here, it
    // will be called when first time ProcessInput() is called. However, if you want to
    // get the actual frame size being used, it should be called explicitly here.
    hr = pDMO->AllocateStreamingResources();
    if (FAILED(hr)){ mexErrMsgTxt("AllocateStreamingResources failed"); }
    
    // Get actually frame size being used in the DMO. (optional, do as you need)
    int iFrameSize;
    PROPVARIANT pvFrameSize;
    PropVariantInit(&pvFrameSize);
    hr=pPS->GetValue(MFPKEY_WMAAECMA_FEATR_FRAME_SIZE, &pvFrameSize);
    if (FAILED(hr)){ mexErrMsgTxt("Error"); }
    iFrameSize = pvFrameSize.lVal;
    PropVariantClear(&pvFrameSize);
  
    hr = pDMO->QueryInterface(IID_ISoundSourceLocalizer, (void**)&pSC);
    if (FAILED(hr)){ mexErrMsgTxt("QueryInterface for IID_ISoundSourceLocalizer failed"); }

	// Output the Point to the Kinect Audio Object
	int Jdimsc[2]={1,9};
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT64_CLASS, mxREAL);
	unsigned __int64 *MXadress;
	MXadress = (unsigned __int64*)mxGetData(plhs[0]);
	
	MXadress[0] = ( unsigned __int64)&wfxOut;
	MXadress[1] = ( unsigned __int64)&pbOutputBuffer;
	MXadress[2] = ( unsigned __int64)&pSC;
	MXadress[3] = ( unsigned __int64)&outputBuffer;
    MXadress[4] = ( unsigned __int64)&pDMO;
	MXadress[5] = bytessecond;
}



