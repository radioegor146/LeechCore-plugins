#include <leechcore.h>
#include <leechcore_device.h>

typedef struct 
{
    HANDLE hSerialPort;
} SM2263XT_CONTEXT, *PSM2263XT_CONTEXT;

_Success_(return) static BOOL
    WritePortFully(_In_ HANDLE hSerialPort, _In_ CONST LPBYTE pData,
    _In_ DWORD dwLength) 
{
    DWORD dwPosition = 0;
    while (dwPosition < dwLength) 
    {
        DWORD dwWrittenBytes = 0;
        if (WriteFile(hSerialPort, pData + dwPosition, dwLength - dwPosition,
                       &dwWrittenBytes, NULL) != TRUE) 
        {
            return FALSE;
        }
        dwPosition += dwWrittenBytes;
    }
    return TRUE;
}


_Success_(return) static BOOL
    ReadPortFully(_In_ HANDLE hSerialPort, _In_ LPBYTE pData,
                   _In_ DWORD dwLength) 
{
    DWORD dwPosition = 0;
    while (dwPosition < dwLength) 
    {
        DWORD dwReadBytes = 0;
        if (ReadFile(hSerialPort, pData + dwPosition, dwLength - dwPosition,
                      &dwReadBytes, NULL) != TRUE) 
        {
            return FALSE;
        }
        dwPosition += dwReadBytes;
    }
    return TRUE;
}

static LPBYTE ConstructPacket(_In_ BYTE bType, CONST LPBYTE pData, _In_ DWORD dwLength, _Out_ PDWORD dwPacketLength)
{
    *dwPacketLength = 2 + 1 + 1 + dwLength + 1;
    LPBYTE pPacket = LocalAlloc(LMEM_ZEROINIT, *dwPacketLength);
    if (pPacket == NULL)
    {
        return NULL;
    }
    pPacket[0] = 0x13;
    pPacket[1] = 0x37;
    pPacket[2] = dwLength + 1;
    pPacket[3] = bType;
    CopyMemory(pPacket + 4, pData, dwLength);

    BYTE bChecksum = 0x37;
    for (DWORD i = 0; i < dwLength + 1; i++)
    {
        bChecksum ^= pPacket[i + 3];
    }
    pPacket[2 + 1 + 1 + dwLength] = bChecksum;

    return pPacket;
}

typedef struct
{
    QWORD qwAddress;
    WORD wSize;
} SM2263XT_READ_PACKET, *PSM2263XT_READ_PACKET;

VOID DeviceSM2263XT_ReadContigious(PLC_READ_CONTIGIOUS_CONTEXT ctxRC) 
{
    PSM2263XT_CONTEXT ctx = (PSM2263XT_CONTEXT)ctxRC->ctxLC->hDevice;

    QWORD qwRealAddress = ctxRC->paBase / 0x200 * 0x200;
    QWORD qwOffset = ctxRC->paBase - qwRealAddress;
    DWORD dwMaxReadSize = ctxRC->cb > 0x1000 ? 0x1000 : ctxRC->cb;
    WORD wRealReadSize = (dwMaxReadSize + 0x200 - 1) / 0x200 * 0x200;

    SM2263XT_READ_PACKET readPacket = {.qwAddress = qwRealAddress,
                                       .wSize = wRealReadSize};
    DWORD dwReadPacketSize = 0;
    LPBYTE pReadPacket = ConstructPacket(1, NULL, 0, &dwReadPacketSize);
    if (pReadPacket == NULL) 
    {
        return;
    }
    if (WritePortFully(ctx->hSerialPort, pReadPacket, dwReadPacketSize) !=
        TRUE) 
    {
        return;
    }
    LocalFree(pReadPacket);
    BYTE bResponse[2] = {0};
    if (ReadPortFully(ctx->hSerialPort, bResponse, sizeof(bResponse)) != TRUE) 
    {
        return;
    }
    if (bResponse[0] != 0x15 || bResponse[1] != 0x39) 
    {
        return;
    }

    BYTE pDataBuffer[0x1000];
    if (ReadPortFully(ctx->hSerialPort, pDataBuffer, wRealReadSize) != TRUE)
    {
        return;
    }

    ctxRC->cbRead = dwMaxReadSize;
    CopyMemory(ctxRC->pb, pDataBuffer + qwOffset, dwMaxReadSize);
}

VOID DeviceSM2263XT_Close(_Inout_ PLC_CONTEXT ctxLC)
{
    PSM2263XT_CONTEXT ctx = (PSM2263XT_CONTEXT)ctxLC->hDevice;
    if (ctx->hSerialPort != NULL &&ctx->hSerialPort != INVALID_HANDLE_VALUE) 
    {
        CloseHandle(ctx->hSerialPort);
    }
}

_Success_(return) BOOL DeviceSM2263XT_TestComms(_Inout_ PSM2263XT_CONTEXT ctx) 
{
    DWORD dwTestPacketSize = 0;
    LPBYTE pTestPacket = ConstructPacket(2, NULL, 0, &dwTestPacketSize);
    if (pTestPacket == NULL)
    {
        return FALSE;
    }
    if (WritePortFully(ctx->hSerialPort, pTestPacket,
                       dwTestPacketSize) != TRUE) 
    {
        return FALSE;
    }
    LocalFree(pTestPacket);
    BYTE bResponse[4] = {0};
    if (ReadPortFully(ctx->hSerialPort, bResponse, sizeof(bResponse)) != TRUE) 
    {
        return FALSE;
    }
    if (bResponse[0] != 0x16 || bResponse[1] != 0x40 || bResponse[2] != 0x17 ||
        bResponse[3] != 0x31) 
    {
        return FALSE;
    }
    return TRUE;
}

_Success_(return) BOOL DeviceSM2263XT_Init(_In_ PLC_CONTEXT ctxLC,
                                               _Inout_ PSM2263XT_CONTEXT ctx) 
{ 
    PLC_DEVICE_PARAMETER_ENTRY pPort = LcDeviceParameterGet(ctxLC, "port");
    QWORD qBaudrate = LcDeviceParameterGetNumeric(ctxLC, "baudrate");
    HANDLE hSerialPort = CreateFile(pPort->szValue, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerialPort == INVALID_HANDLE_VALUE) 
    {
        return FALSE;
    }

    ctx->hSerialPort = hSerialPort;

    DCB dcb;

    FillMemory(&dcb, sizeof(dcb), 0);
    if (!GetCommState(hSerialPort, &dcb)) 
    {
        return FALSE;
    }

    dcb.BaudRate = qBaudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = 1;
    dcb.Parity = PARITY_NONE;

    if (!SetCommState(hSerialPort, &dcb))
    {
        return FALSE;
    }

    if (!DeviceSM2263XT_TestComms(ctx)) 
    {
        return FALSE;    
    }

    return TRUE;
}

_Success_(return)
EXPORTED_FUNCTION BOOL LcPluginCreate(_Inout_ PLC_CONTEXT ctxLC, _Out_opt_ PPLC_CONFIG_ERRORINFO ppLcCreateErrorInfo)
{
    PSM2263XT_CONTEXT ctx = NULL;
    if(ppLcCreateErrorInfo) { *ppLcCreateErrorInfo = NULL; }
    if(ctxLC->version != LC_CONTEXT_VERSION) { return FALSE; }
    if (!(ctx = (PSM2263XT_CONTEXT)LocalAlloc(LMEM_ZEROINIT,
                                              sizeof(SM2263XT_CONTEXT)))) {
        return FALSE;
    }

    if(!DeviceSM2263XT_Init(ctxLC, ctx)) {
        lcprintf(ctxLC, "DEVICE: FAILED: SM2263XT init failed");
        goto fail;
    }

    // set callback functions and fix up config
    ctxLC->hDevice = (HANDLE)ctx;
    ctxLC->Config.fVolatile = FALSE;
    ctxLC->pfnClose = DeviceSM2263XT_Close;
    ctxLC->pfnReadContigious = DeviceSM2263XT_ReadContigious;
    ctxLC->ReadContigious.cThread = 1;
    lcprintfv(ctxLC, "DEVICE: Successfully SM2263XT '%s'.\n", ctxLC->Config.szDevice);
    return TRUE;
fail:
    ctxLC->hDevice = (HANDLE)ctx;
    DeviceSM2263XT_Close(ctxLC);
    return FALSE;
}
