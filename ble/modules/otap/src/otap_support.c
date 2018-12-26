
#include "otap_support.h"
#include <string.h>

/************************************************************************************
*  Updates the CRC based on the received data to process.
*  Updates the global CRC value. This was determined to be optimal from a resource
*  consumption POV.
*
*  Input parameters:
*  - None
*  Return:
*  - None
************************************************************************************/
uint16_t OTA_CrcCompute(uint8_t *pData, uint32_t lenData, uint16_t crcValueOld)
{
    uint8_t i;

    while (lenData--)
    {
        crcValueOld ^= (uint16_t)((uint16_t)*pData++ << 8);
        for (i = 0; i < 8; ++i)
        {
            if (crcValueOld & 0x8000)
            {
                crcValueOld = (crcValueOld << 1) ^ 0x1021U;
            }
            else
            {
                crcValueOld = crcValueOld << 1;
            }
        }
    }
    return crcValueOld;
}

bool OtapClient_ImageInformationExchange(uint8_t *pRemoteImgId,
                                         uint8_t *pRemoteImgVer,
                                         const uint8_t *pCurrentImgId,
                                         const uint8_t *pcurrentImgVer)
{
    QPRINTF("OtapClient_ImageInformationExchange\r\n");
    uint32_t remoteBuildVer;
    uint32_t localeBuildVer;
    /*! Ignore the Image Id for the moment. */
    /*! Check the Manufacturer Id */
    if (pRemoteImgVer[7] != pcurrentImgVer[7])
    {
        return false;
    }

    /*! Check Hardware Id */
    if (memcmp((void *)(&(pRemoteImgVer[4])), (void *)(&(pcurrentImgVer[4])), 3) > 0)
    {
        return false;
    }

    /*! Check Stack Version */
    if (pRemoteImgVer[3] < pcurrentImgVer[3])
    {
        return false;
    }

    /*! Check Build Version */
    remoteBuildVer =
        (uint32_t)pRemoteImgVer[0] + ((uint32_t)(pRemoteImgVer[1]) << 8) + ((uint32_t)(pRemoteImgVer[2]) << 16);
    localeBuildVer =
        (uint32_t)pcurrentImgVer[0] + ((uint32_t)(pcurrentImgVer[1]) << 8) + ((uint32_t)(pcurrentImgVer[2]) << 16);
    if (remoteBuildVer <= localeBuildVer)
    {
        return false;
    }

    return true;
}

otapStatus_t OtapClient_IsImageFileHeaderValid(bleOtaImageFileHeader_t *imgFileHeader)
{
    if (imgFileHeader->fileIdentifier != gBleOtaFileHeaderIdentifier_c)
    {
        return gOtapStatusUnknownFileIdentifier_c;
    }

    if (imgFileHeader->headerVersion != gbleOtapHeaderVersion0100_c)
    {
        return gOtapStatusUnknownHeaderVersion_c;
    }

    if (imgFileHeader->headerLength != sizeof(bleOtaImageFileHeader_t))
    {
        return gOtapStatusUnexpectedHeaderLength_c;
    }

    if (imgFileHeader->fieldControl != gBleOtaFileHeaderDefaultFieldControl_c)
    {
        return gOtapStatusUnexpectedHeaderFieldControl_c;
    }

    if (imgFileHeader->companyId != gBleOtaCompanyIdentifier_c)
    {
        return gOtapStatusUnknownCompanyId_c;
    }

    return gOtapStatusSuccess_c;
}

void OtapServer_ImageInfoProc(uint8_t *pRemoteImgId,
                              uint8_t *pRemoteImgVer,
                              const uint8_t *pCurrentImgId,
                              const uint8_t *pcurrentImgVer)
{
    QPRINTF("\r\n Please load another image if this image is not suitable for client.\r\n");
    return;
}

uint32_t OtapClient_ReceivedFileType(bleOtaImageFileSubElementTagId_t tag)
{
    uint32_t flashDstAddr = FLASH_IMG0_SWAP_OFFSET;
    switch (tag)
    {
        // image start address, this address shall not modified by user
        case gBleOtaSubElemTagIdUpgradeImage_c:
            flashDstAddr = FLASH_IMG0_SWAP_OFFSET;
            break;

        // user set data address here
        case gBleOtaSubElemTagIdUserData_c:
            flashDstAddr = FLASH_IMG0_SWAP_OFFSET;
            break;
    }

    return flashDstAddr;
}

void Otap_TransmitedImagePercent(int percent)
{
    QPRINTF("\x08\x08\x08\x08");
    QPRINTF("%3d%%", percent);
}
