/********************************************************//**
* \file usb_app.c
* \version 1.0
*
* \brief Implements the USB data handling part of the Slave FIFO application.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usb_common.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "usb_app.h"
#include "cy_debug.h"
#include "usb_i2c.h"
#include "usb_qspi.h"
#include "cy_lvds.h"

/* Whether SET_CONFIG is complete or not. */
static volatile bool glIsDevConfigured = false;
static volatile bool cy_IsApplnActive = false;
bool glIsFPGARegConfigured = false;
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;

#if FPGA_ENABLE
/**
 * \name Cy_Slff_ConfigFpgaRegister
 * \brief   FPGA Register Writes. FPGA is configured to send internally generated 
 *          colorbar data over FX2G3's SlaveFIFO Interface
 * \retval 0 for read success, error code for unsuccess.
 */
static cy_en_scb_i2c_status_t Cy_Slff_ConfigFpgaRegister (void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    /* Disable device0 before configuring FPGA register */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+FPGA_DEVICE_STREAM_ENABLE_ADDRESS,DATA_DISABLE,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH); 
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    
    /* Disable device1 before configuring FPGA register */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+FPGA_DEVICE_STREAM_ENABLE_ADDRESS,DATA_DISABLE,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH); 
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /* write FPGA register to enable UVC */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_UVC_SELECTION_ADDRESS,FPGA_UVC_ENABLE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_HEADER_CTRL_ADDRESS,FPGA_HEADER_ENABLE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

    /* Number of active device list - 2nos*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_ACTIVE_DIVICE_MASK_ADDRESS,0x03,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);  


    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);
    DBG_APP_INFO("Register Writes for Device 0 \n\r");

    /* Write FPGA register to disable format converstion */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+FPGA_DEVICE_STREAM_MODE_ADDRESS,NO_CONVERSION,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_TYPE_ADDRESS,INTERNAL_COLORBAR,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    
    /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_ACTIVE_TREAD_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 2 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /* Thread 1 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD1_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 1 socket information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD1_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Clear FPGA register during power up, this will get update when firmware detects HDMI */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_INFO_ADDRESS,SOURCE_DISCONNECT,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    DBG_APP_INFO("DMA Buffer Size %d \n\r",FPGA_DMA_BUFFER_SIZE);
 
    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_BUFFER_SIZE_MSB_ADDRESS,CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_BUFFER_SIZE_LSB_ADDRESS,CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH); 
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /*Register writes for device 1*/
    DBG_APP_INFO("Register Writes for Device 1\n\r");

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+FPGA_DEVICE_STREAM_MODE_ADDRESS,NO_CONVERSION,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_SOURCE_TYPE_ADDRESS,INTERNAL_COLORBAR,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    
    /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_ACTIVE_TREAD_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_THREAD2_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 2 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_THREAD2_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /* Thread 1 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_THREAD1_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 1 socket information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_THREAD1_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_SOURCE_INFO_ADDRESS,SOURCE_DISCONNECT,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    DBG_APP_INFO("DMA Buffer Size %d \n\r",FPGA_DMA_BUFFER_SIZE);
 
    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_BUFFER_SIZE_MSB_ADDRESS,CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE1_OFFSET+DEVICE_BUFFER_SIZE_LSB_ADDRESS,CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH); 
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    return status;
} //End of Cy_Slff_ConfigFpgaRegister()

/**
 * \name Cy_Slff_DataStreamStartStop
 * \brief Starts/Stops FPGA data streaming
 * \param device_offset device number (fpga implementation specific)
 * \param IsStreamStart Pass 1 to start streaming from fpga elase 0
 * \retval  0 for read success, error code for unsuccess.
 */
cy_en_scb_i2c_status_t Cy_Slff_DataStreamStartStop(uint8_t device_offset, uint8_t IsStreamStart)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    if (device_offset == DEVICE0_OFFSET)
        cy_IsApplnActive = IsStreamStart?true:false;

    if(IsStreamStart)
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),DATA_ENABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    else

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),DATA_DISABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    DBG_APP_INFO("cy_IsApplnActive = 0x%x\n\r",cy_IsApplnActive);

    if(false == status)
    {
        DBG_APP_INFO("App = %d device_offset 0x%x\n\r",IsStreamStart, device_offset);
    }

    return status;
} /* End of Cy_Slff_DataStreamStartStop() */
#endif

/**
 * \name Cy_Slff_AppStop
 * \brief Stop the data stream channels
 * \param pAppCtxt application layer context pointer
 * \param pUsbdCtxt USBD layer context pointer
 * \param IsStreamStart Pass 1 to start streaming from fpga elase 0
 * \retval None
 */
static void Cy_Slff_AppStop(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t epNumber)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint8_t index = 0;
    cy_stc_hbdma_sock_t sckStat;

    DBG_APP_INFO("App Stop:epNumber=0x%x\r\n",epNumber);

    if (BULK_OUT_ENDPOINT_1 == epNumber)
    {
        for(index = 0; index < pAppCtxt->hbBulkOutChannel[0]->prodSckCount; index++)
        {
            do {
                Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, pAppCtxt->hbBulkOutChannel[0]->prodSckId[index], &sckStat);
            } while(((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE,sckStat.status)) != 0x1));

            DBG_APP_INFO("DMA Socket %x is stalled\r\n", pAppCtxt->hbBulkOutChannel[0]->prodSckId[index]);
        }
#if FPGA_ENABLE
        Cy_Slff_DataStreamStartStop(DEVICE0_OFFSET, STOP);
#endif /* FPGA_ENABLE */
        /* Reset the DMA channel through which data is received from the LVDS side. */
        status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkOutChannel[0]);
        ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);

        DBG_APP_INFO("EP: %d DMA Reset and App Stoppedr\n",epNumber);
    }
    else if (BULK_OUT_ENDPOINT_2 == epNumber)
    {
        for(index = 0; index < pAppCtxt->hbBulkOutChannel[1]->prodSckCount; index++)
        {
            do {
                Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, pAppCtxt->hbBulkOutChannel[1]->prodSckId[index], &sckStat);
            } while(((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE,sckStat.status)) != 0x1));

            DBG_APP_INFO("DMA Socket %x is stalled\r\n", pAppCtxt->hbBulkOutChannel[1]->prodSckId[index]);
        }

#if FPGA_ENABLE
        Cy_Slff_DataStreamStartStop(DEVICE1_OFFSET, STOP);
#endif /* FPGA_ENABLE */

        /* Reset the DMA channel through which data is received from the LVDS side. */
        status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkOutChannel[1]);
        ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);

        DBG_APP_INFO("EP: %d DMA Reset and App Stoppedr\n",epNumber);
    }

    /* On USB 2.0 connection, reset the DataWire channel used to send data to the EPM. */
    Cy_USBHS_App_ResetEpDma(&(pAppCtxt->endpInDma[epNumber]));

    /* Flush and reset the endpoint and clear the STALL bit. */
    Cy_USBD_FlushEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN);
    Cy_USBD_ResetEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN, false);
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, (cy_en_usb_endp_dir_t)epNumber, CY_USB_ENDP_DIR_IN, false);

}

/**
 * \name Cy_Slff_AppHandleTxEvent
 * \brief Function to handle data transmitted on slave fifo interface
 * \param pUsbApp application layer context pointer
 * \param pChHandle HBDMA channel handler
 * \param incFlag flag to queue data on USB
 * \param index slave fifo channel index
 * \retval void
 */
static void Cy_Slff_AppHandleTxEvent (cy_stc_usb_app_ctxt_t  *pUsbApp, cy_stc_hbdma_channel_t *pChHandle, uint8_t index)
{
    if (index == 1)
    {
        pUsbApp->slffTxBufFullCount1--;
        if((pUsbApp->slffTxBufFullCount1 < SLFF_TX_MAX_BUFFER_COUNT) && (pUsbApp->slffReadInQueue1 == false))
        {
            Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum1, pUsbApp->pSlffTxBuffer1[pUsbApp->slffNextTxBufIndex1], SLFF_TX_MAX_BUFFER_SIZE,index);
        }
    }
    else if (index == 2)
    {
        pUsbApp->slffTxBufFullCount2--;
        if((pUsbApp->slffTxBufFullCount2 < SLFF_TX_MAX_BUFFER_COUNT) && (pUsbApp->slffReadInQueue2 == false))
        {
            Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum2, pUsbApp->pSlffTxBuffer2[pUsbApp->slffNextTxBufIndex2], SLFF_TX_MAX_BUFFER_SIZE,index);
        }

    }

}

/**
 * \name Cy_Slff_AppHbDmaTxCallback
 * \brief Callback function for the channel transmitting data out of LVDS socket
 * \param handle HBDMA channel handle
 * \param cy_en_hbdma_cb_type_t HBDMA channel type
 * \param pbufStat fHBDMA buffer status
 * \param userCtx suser context
 * \retval void
 */
void Cy_Slff_AppHbDmaTxCallback(
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;
    uint8_t index = 0;


    if(handle == pAppCtxt->hbBulkOutChannel[0])
    {
        index = 1;
    }
    else if (handle == pAppCtxt->hbBulkOutChannel[1])
    {
        index = 2;
    }

    if (type == CY_HBDMA_CB_CONS_EVENT)
    {   
        Cy_Slff_AppHandleTxEvent(pAppCtxt, handle,index);   
    }
}

/**
 * \name Cy_Slff_AppTaskHandler
 * \brief Application task handler
 * \param pTaskParam task param
 * \retval void
 */
void Cy_Slff_AppTaskHandler(void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_usbd_app_msg_t queueMsg;
    BaseType_t xStatus;
    uint32_t lpEntryTime = 0;
    uint8_t i ;

    vTaskDelay(250);

#if FPGA_ENABLE

    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt,&HBW_BufMgr);
    Cy_SPI_FlashInit(SPI_FLASH_0, true,false);

    DBG_APP_INFO("Configure FPGA\n\r");
    Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE);

    if(!glIsFPGARegConfigured)
    {
        Cy_APP_GetFPGAVersion(pAppCtxt);
        if(0 == Cy_Slff_ConfigFpgaRegister())
        {
            glIsFPGARegConfigured = true;
            DBG_APP_INFO("Successfuly configured the FPGA via I2C \n\r"); 
        }
        else
        {
            LOG_ERROR("Failed to configure FPGA via I2C \r\n");
        }
    }
#endif

    /* Initialize the LVDS interface. */
    Cy_Slff_LvdsInit(); 
    vTaskDelay(100);

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        Cy_USB_EnableUsbHSConnection(pAppCtxt);
    }

    DBG_APP_INFO("AppThreadCreated\r\n");

    for (;;)
    {

        /*
         * If the link has been in USB2-L1 for more than 0.5 seconds, initiate LPM exit so that
         * transfers do not get delayed significantly.
         */
        if ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
        {
            if ((Cy_USBD_GetTimerTick() - lpEntryTime) >= 500UL) {
                lpEntryTime = Cy_USBD_GetTimerTick();
                Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
            }
        } else {
            lpEntryTime = Cy_USBD_GetTimerTick();
        }

        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->usbMsgQueue, &queueMsg, 100);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {

            case CY_USB_UVC_VBUS_CHANGE_INTR:
                /* Start the debounce timer. */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_UVC_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed. */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_EnableUsbHSConnection(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        /* On USB 2.x connections, make sure the DataWire channels are disabled and reset. */
 
                        for (i = 1; i < CY_USB_MAX_ENDP_NUMBER; i++) {
                            if (pAppCtxt->endpInDma[i].valid) {
                                /* DeInit the DMA channel and disconnect the triggers. */
                                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
                            }

                            if (pAppCtxt->endpOutDma[i].valid) {
                                /* DeInit the DMA channel and disconnect the triggers. */
                                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpOutDma[i]));
                            }
                        }

                        if (pAppCtxt->hbChannelCreated)
                        {
                            DBG_APP_TRACE("HBDMA destroy\r\n");
                            pAppCtxt->hbChannelCreated = false;
                            Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkOutChannel[0]);
                            Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkOutChannel[0]);
                            Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkOutChannel[1]);
                            Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkOutChannel[1]);
                        }
                    }
                    DBG_APP_INFO("Disabling USB connection due to VBus removal\r\n");
                    Cy_USB_DisableUsbHSConnection(pAppCtxt);
                }
                
            break;
            case CY_USB_STREAMING_START:
                DBG_APP_INFO("CY_USB_STREAMING_START for EP %x\r\n", (uint8_t)queueMsg.data[0]);
                if((uint8_t)(queueMsg.data[0]) == BULK_OUT_ENDPOINT_1)
                {
#if FPGA_ENABLE
                    Cy_Slff_DataStreamStartStop(DEVICE0_OFFSET, START);
#endif
                }
                else if((uint8_t)(queueMsg.data[0]) == BULK_OUT_ENDPOINT_2)
                {
#if FPGA_ENABLE
                    Cy_Slff_DataStreamStartStop(DEVICE1_OFFSET, START);
#endif
                }
            break;

            case CY_USB_STREAMING_STOP:
                DBG_APP_INFO("CY_USB_STREAMING_STOP for EP %x\r\n", (uint8_t)queueMsg.data[0]);
                if((uint8_t)(queueMsg.data[0]) == BULK_OUT_ENDPOINT_1)
                {
                    Cy_Slff_AppStop(pAppCtxt, pAppCtxt->pUsbdCtxt,BULK_OUT_ENDPOINT_1);
                }
                else if((uint8_t)(queueMsg.data[0]) == BULK_OUT_ENDPOINT_2)
                {
                    Cy_Slff_AppStop(pAppCtxt, pAppCtxt->pUsbdCtxt,BULK_OUT_ENDPOINT_2);
                }
            break;
            default:
            break;
        }

       
    } /* End of for(;;) */
}

/**
 * \name Cy_USB_VbusDebounceTimerCallback
 * \brief Timer used to do debounce on VBus changed interrupt notification.
 * \param xTimer Timer Handle
 * \retval None
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("VbusDebounce_CB\r\n");
    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete. */
        xMsg.type = CY_USB_UVC_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt. */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

/**
 * \name Cy_USB_AppInit
 * \brief   This function Initializes application related data structures, register callback
 *          creates task for device function.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer Context pointer
 * \param pCpuDmacBase DMAC base address
 * \param pCpuDw0Base DataWire 0 base address
 * \param pCpuDw1Base DataWire 1 base address
 * \param pHbDmaMgrCtxt HBDMA Manager Context
 * \retval None
 */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;
    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;
    pAppCtxt->hbChannelCreated = false;

    for (index = 0x00; index < CY_USB_MAX_ENDP_NUMBER; index++)
    {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone))
    {

        /* Create the message queue and register it with the kernel. */
        pAppCtxt->usbMsgQueue = xQueueCreate(CY_USB_DEVICE_MSG_QUEUE_SIZE,
                CY_USB_DEVICE_MSG_SIZE);
        if (pAppCtxt->usbMsgQueue == NULL) {
            DBG_APP_ERR("QueuecreateFail\r\n");
            return;
        }

        vQueueAddToRegistry(pAppCtxt->usbMsgQueue, "DeviceMsgQueue");
        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_Slff_AppTaskHandler, "SlaveFIFODeviceTask", 2048,
                             (void *)pAppCtxt, 5, &(pAppCtxt->slffTaskHandle));

        if (status != pdPASS)
        {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }

        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);
        if (pAppCtxt->vbusDebounceTimer == NULL) {
            DBG_APP_ERR("TimerCreateFail\r\n");
            return;
        }
        DBG_APP_INFO("VBus debounce timer created\r\n");
        pAppCtxt->firstInitDone = 0x01;
    }
} /* end of function. */

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief  This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer.
 * \retval None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, Cy_USB_AppBusResetDoneCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, Cy_USB_AppBusSpeedCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, Cy_USB_AppSuspendCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, Cy_USB_AppResumeCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, Cy_USB_AppSetCfgCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, Cy_USB_AppSetIntfCallback);
	
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP, Cy_USB_AppSlpCallback);
}

/**
 * \name Cy_USB_AppSetupEndpDmaParamsHs
 * \brief  Configure and enable HBW DMA channels.
 * \param pAppCtxtapplication layer context pointer.
 * \param pEndpDscrEndpoint descriptor pointer
 * \retval None
 */
static void Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t mgrStat;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    DW_Type *pDW;
    bool stat;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);
    
    if (endpNumber == BULK_OUT_ENDPOINT_1)
    {
        pUsbApp->slffOutEpNum1    = (uint8_t)endpNumber;
        pUsbApp->slffNextTxBufIndex1 = 0;
        pUsbApp->slffTxBufFullCount1 = 0;
        dmaConfig.size          = SLFF_TX_MAX_BUFFER_SIZE;      /* DMA Buffer Size in bytes */
        dmaConfig.count         = SLFF_TX_MAX_BUFFER_COUNT;     /* DMA Buffer Count */
        dmaConfig.bufferMode    = true;                         /* DMA buffer mode disabled */
        dmaConfig.prodHdrSize   = 0;
        dmaConfig.prodBufSize   = SLFF_TX_MAX_BUFFER_SIZE;    
        dmaConfig.eventEnable   = 0;                            /* Enable for DMA AUTO */
        dmaConfig.intrEnable    = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk | 
                                    LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.prodSckCount  = 1;                            /* No. of producer sockets */
        dmaConfig.consSckCount  = 1;                            /* No. of consumer Sockets */
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;      /* Producer Socket ID: None */
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;      /* Consumer Socket ID: None */
        dmaConfig.cb            = Cy_Slff_AppHbDmaTxCallback;   /* HB-DMA callback */
        dmaConfig.userCtx       = (void *)(pUsbApp);            /* Pass the application context as user context */
        dmaConfig.consSck[0]    = CY_HBDMA_LVDS_SOCKET_00;
        dmaConfig.chType     = CY_HBDMA_TYPE_MEM_TO_IP;
        dmaConfig.prodSck[0] = (cy_hbdma_socket_id_t)0;
        
        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr, &pUsbApp->endpOutDma[endpNumber].hbDmaChannel, &dmaConfig);

        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("BulkOut channel create failed 0x%x\r\n", mgrStat);
            return;
        }
        else
        {
            /* Store the DMA channel pointer. */
            pUsbApp->hbBulkOutChannel[0] = &(pUsbApp->endpOutDma[endpNumber].hbDmaChannel);
            /* Get the list of DMA buffers to be used for LVDS transfers. */
            Cy_HBDma_Channel_GetBufferInfo(pUsbApp->hbBulkOutChannel[0], pUsbApp->pSlffTxBuffer1, 4);
            mgrStat = Cy_HBDma_Channel_Enable(pUsbApp->hbBulkOutChannel[0], 0);
            DBG_APP_INFO("OutChnEnable status: %x\r\n", mgrStat);
        }
    }
    else if (endpNumber == BULK_OUT_ENDPOINT_2)
    {
        pUsbApp->slffOutEpNum2    = (uint8_t)endpNumber;
        pUsbApp->slffNextTxBufIndex2 = 0;
        pUsbApp->slffTxBufFullCount2 = 0;
        dmaConfig.size          = SLFF_TX_MAX_BUFFER_SIZE;      /* DMA Buffer Size in bytes */
        dmaConfig.count         = SLFF_TX_MAX_BUFFER_COUNT;     /* DMA Buffer Count */
        dmaConfig.bufferMode    = true;                         /* DMA buffer mode disabled */
        dmaConfig.prodHdrSize   = 0;
        dmaConfig.prodBufSize   = SLFF_TX_MAX_BUFFER_SIZE;    
        dmaConfig.eventEnable   = 0;                            /* Enable for DMA AUTO */
        dmaConfig.intrEnable    = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk | 
                                    LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.prodSckCount  = 1;                            /* No. of producer sockets */
        dmaConfig.consSckCount  = 1;                            /* No. of consumer Sockets */
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;      /* Producer Socket ID: None */
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;      /* Consumer Socket ID: None */
        dmaConfig.cb            = Cy_Slff_AppHbDmaTxCallback;   /* HB-DMA callback */
        dmaConfig.userCtx       = (void *)(pUsbApp);            /* Pass the application context as user context */
        dmaConfig.consSck[0]    = CY_HBDMA_LVDS_SOCKET_01;
        dmaConfig.chType     = CY_HBDMA_TYPE_MEM_TO_IP;
        dmaConfig.prodSck[0] = (cy_hbdma_socket_id_t)0;
        
        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr, &pUsbApp->endpOutDma[endpNumber].hbDmaChannel, &dmaConfig);

        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("BulkOut channel create failed 0x%x\r\n", mgrStat);
            return;
        }
        else
        {
            /* Store the DMA channel pointer. */
            pUsbApp->hbBulkOutChannel[1] = &(pUsbApp->endpOutDma[endpNumber].hbDmaChannel);
            /* Get the list of DMA buffers to be used for LVDS transfers. */
            Cy_HBDma_Channel_GetBufferInfo(pUsbApp->hbBulkOutChannel[1], pUsbApp->pSlffTxBuffer2, 4);
            mgrStat = Cy_HBDma_Channel_Enable(pUsbApp->hbBulkOutChannel[1], 0);
            DBG_APP_INFO("OutChnEnable status: %x\r\n", mgrStat);
        }
    }

    /* Set flag to indicate DMA channels have been created. */
    pUsbApp->hbChannelCreated = true;

    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
    {
        dir = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);
        pDW = pUsbApp->pCpuDw1Base;
    }
    else
    {
        dir = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);
        pDW = pUsbApp->pCpuDw0Base;
    }

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber,(cy_en_usb_endp_dir_t)dir, maxPktSize);
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, dir, stat);

    return;
} /* end of function  */


/**
 * \name Cy_USB_AppConfigureEndp
 * \brief  Configure all endpoints used by application (except EP0)
 * \param pUsbdCtxt USBD layer context pointer
 * \param pEndpDscr Endpoint descriptor pointer
 * \retval None
 */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t interval = 0x00;
    cy_en_usbd_ret_code_t usbdRetCode;

    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr))
    {
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir)
    {
        endpDirection = CY_USB_ENDP_DIR_IN;
    }
    else
    {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType))
    {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK) >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    Cy_USBD_GetEndpInterval(pEndpDscr, &interval);

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.interval = interval;
    /*
     * allowNakTillDmaRdy = true means device will send NAK
     * till DMA setup is ready. This field is applicable to only
     * ingress direction ie OUT transfer/OUT endpoint.
     * For Egress ie IN transfer, this field is ignored.
     */
    endpConfig.allowNakTillDmaRdy = TRUE;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("#ENDPCFG: %d, %d\r\n", endpNumber, usbdRetCode);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetCfgCallback
 * \brief Callback function will be invoked by USBD when set configuration is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    glIsDevConfigured = true;
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DBG_APP_INFO("AppSetCfgCbStart\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    
    /*
     * Based on type of application as well as how data flows,
     * data wire can be used so initialize datawire.
     */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg)
    {
        /* Set config should be called when active config value > 0x00. */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00)
    {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++)
    {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL)
        {
            DBG_APP_INFO("pIntfDscrNull\r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00)
        {
            DBG_APP_INFO("numOfEndp 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            
        }
    }

#if CY_CPU_CORTEX_M4
    Cy_USB_AppInitDmaIntr(BULK_OUT_ENDPOINT_1, CY_USB_ENDP_DIR_OUT, Cy_Slff_TxChannel1DataWire_ISR);
    Cy_USB_AppInitDmaIntr(BULK_OUT_ENDPOINT_2, CY_USB_ENDP_DIR_OUT, Cy_Slff_TxChannel2DataWire_ISR);
#else
    Cy_USB_AppInitDmaIntr(BULK_OUT_ENDPOINT_1, CY_USB_ENDP_DIR_OUT, Cy_Slff_TxDataWireCombined_ISR);
    Cy_USB_AppInitDmaIntr(BULK_OUT_ENDPOINT_2, CY_USB_ENDP_DIR_OUT, Cy_Slff_TxDataWireCombined_ISR);
#endif
    /* Queue read on USB OUT endpoint */
    Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum1,
                        pUsbApp->pSlffTxBuffer1[pUsbApp->slffNextTxBufIndex1],
                        SLFF_TX_MAX_BUFFER_SIZE,1);
    /* Queue read on USB OUT endpoint */
    Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum2,
                        pUsbApp->pSlffTxBuffer2[pUsbApp->slffNextTxBufIndex2],
                        SLFF_TX_MAX_BUFFER_SIZE,2);
    

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    /* Enable data from FPGA*/
    if(pUsbApp->slffOutEpNum1 == BULK_OUT_ENDPOINT_1)
    {
        DBG_APP_INFO("Enable Device - 0\r\n");
        xMsg.type = CY_USB_STREAMING_START;
        xMsg.data[0] = BULK_OUT_ENDPOINT_1;
        xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    }
    if (pUsbApp->slffOutEpNum2 == BULK_OUT_ENDPOINT_2)
    {    
        DBG_APP_INFO("Enable Device - 1\r\n");
        xMsg.type = CY_USB_STREAMING_START;
        xMsg.data[0] = BULK_OUT_ENDPOINT_2;
        xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    }

    Cy_USBD_LpmDisable(pUsbdCtxt);


    DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function will be invoked by USBD when bus detects RESET
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppBusResetCallback\r\n");

    /* Stop and destroy the high bandwidth DMA channel if present. To be done before AppInit is called. */
    if (pUsbApp->hbChannelCreated)
    {
        DBG_APP_TRACE("HBDMA destroy\r\n");
        pUsbApp->hbChannelCreated = false;
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkOutChannel[0]);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkOutChannel[0]);
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkOutChannel[1]);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkOutChannel[1]);
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase, pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base, pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    return;
} /* end of function. */

/**
 * \name Cy_USB_AppBusResetDoneCallback
 * \brief Callback function will be invoked by USBD when RESET is completed
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("ppBusResetDoneCallback\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
} /* end of function. */

/**
 * \name Cy_USB_AppBusSpeedCallback
 * \brief   Callback function will be invoked by USBD when speed is identified or
 *          speed change is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    return;
} /* end of function. */

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    bool isReqHandled = false;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    BaseType_t status = 0;

    DBG_APP_INFO("AppSetupCallback\r\n");

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_INFO("CY_USB_CTRL_REQ_STD\r\n");
        if (bRequest == CY_USB_SC_SET_FEATURE)
        {
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
                isReqHandled = true;
            }

            /* SET-FEATURE(EP-HALT) is only supported to facilitate Chapter 9 compliance tests. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                        epDir, true);

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if (bRequest == CY_USB_SC_CLEAR_FEATURE)
        {
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                if ((epDir == CY_USB_ENDP_DIR_OUT) && ((((uint32_t)wIndex & 0x7FUL) == BULK_OUT_ENDPOINT_1)|| ((((uint32_t)wIndex & 0x7FUL) == BULK_OUT_ENDPOINT_2))))
                {
                    /* For any EP other than the SLFF streaming endpoint, just clear the STALL bit. */
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                            epDir, false);
                    
                    if(((uint32_t)wIndex & 0x7FUL) == BULK_OUT_ENDPOINT_1)
                    {
                        DBG_APP_INFO("Disable Device - 0\r\n");
                        xMsg.type = CY_USB_STREAMING_STOP;
                        xMsg.data[0] = BULK_OUT_ENDPOINT_1;
                        status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                        ASSERT_NON_BLOCK(pdTRUE == status,status);
                    }
                    else if (((uint32_t)wIndex & 0x7FUL) == BULK_OUT_ENDPOINT_2)
                    {    
                        DBG_APP_INFO("Disable Device - 1\r\n");
                        xMsg.type = CY_USB_STREAMING_STOP;
                        xMsg.data[0] = BULK_OUT_ENDPOINT_2;
                        status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                        ASSERT_NON_BLOCK(pdTRUE == status,status);
                    }
                }
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        /* Handle Microsoft OS String Descriptor request. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

            /* Make sure we do not send more data than requested. */
            if (wLength > glOsString[0]) {
                wLength = glOsString[0];
            }

            DBG_APP_INFO("OSString\r\n");
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsString, wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        }

    }

    if (bType == CY_USB_CTRL_REQ_VENDOR) {
        /* If trying to bind to WinUSB driver, we need to support additional control requests. */
        /* Handle OS Compatibility and OS Feature requests */

        if (bRequest == MS_VENDOR_CODE) {
            if (wIndex == 0x04) {
                if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                    wLength = *((uint16_t *)glOsCompatibilityId);
                }

                DBG_APP_INFO("OSCompat\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsCompatibilityId, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
            else if (wIndex == 0x05) {
                if (wLength > *((uint16_t *)glOsFeature)) {
                    wLength = *((uint16_t *)glOsFeature);
                }

                DBG_APP_INFO("OSFeature\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsFeature, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
        }

        if (isReqHandled) {
            return;
        }
    }

    /* If Request is not handled by the callback, Stall the command */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function. */

/**
 * \name Cy_USB_AppSuspendCallback
 * \brief Callback function will be invoked by USBD when Suspend signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function. */

/**
 * \name Cy_USB_AppResumeCallback
 * \brief Callback function will be invoked by USBD when Resume signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppResumeCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState = pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
} /* end of function. */

/**
 * \name Cy_USB_AppSetIntfCallback
 * \brief Callback function will be invoked by USBD when SET_INTERFACE is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppSetIntfCallback Start\r\n");
    pSetupReq = &(pUsbdCtxt->setupReq);
    /*
     * Get interface and alt setting info. If new setting same as previous
     * then return.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

    if (altSetting == pUsbApp->prevAltSetting)
    {
        DBG_APP_INFO("SameAltSetting\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->prevAltSetting);
    DBG_APP_INFO("unconfigPrevAltSet\r\n");
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("SetIntf:prevNumEp 0\r\n");
    }
    else
    {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
            {
                endpDirection = CY_USB_ENDP_DIR_IN;
            }
            else
            {
                endpDirection = CY_USB_ENDP_DIR_OUT;
            }
            endpNumber =
                (uint32_t)((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

            /* with FALSE, unconfgure previous settings. */
            Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, FALSE);

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    /* Now take care of different config with new alt setting. */
    pUsbApp->prevAltSetting = altSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("SetIntf:numEp 0\r\n");
    }
    else
    {
        pUsbApp->prevAltSetting = altSetting;
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    DBG_APP_INFO("AppSetIntfCallback done\r\n");
    return;
} /* end of function. */

/**
 * \name Cy_USB_AppReadShortPacket
 * \brief Function to modify an ongoing DMA read operation to take care of a short packet.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USB endpoint number
 * \param pktSize USB data size
 * \retval Total size of data in the DMA buffer including data which was already read by the channel.
 */
uint16_t
Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p=NULL;
    uint16_t dataSize = 0;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) || (pAppCtxt->pCpuDw0Base == NULL))
    {
        DBG_APP_ERR("ReadSLP:NULL\r\n");
        return 0;
    }

    /* Verify that the selected endpoint is valid. */
    if (pAppCtxt->endpOutDma[endpNumber].valid == 0)
    {
        DBG_APP_ERR("ReadSLP:BadParam\r\n");
        return 0;
    }

    dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);

    if (dmaset_p->endpType != CY_USB_ENDP_TYPE_ISO) {
        /* NAK the endpoint until we queue a new DMA request. */
        Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
    }

    /* The code assumes that the channel is active. */
    dataSize = Cy_USBHS_App_ReadShortPacket(dmaset_p, pktSize);
    

    return dataSize;
} /* end of function */

/**
 * \name Cy_USB_AppSlpCallback
 * \brief This function will be called by USBD layer when SLP message is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;    
    uint16_t commitSize = 0;

    if(pAppCtxt->slffOutEpNum1 == pMsg->data[0])
    {
        pAppCtxt->slffSlpRx1 = true;
        commitSize = Cy_USB_AppReadShortPacket(pAppCtxt, pAppCtxt->slffOutEpNum1, pMsg->data[1]);
        pAppCtxt->slffTxBufferCount1[pAppCtxt->slffNextTxBufIndex1] = commitSize;
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + pAppCtxt->slffOutEpNum1, CY_TRIGGER_TWO_CYCLES);
    }
    else if (pAppCtxt->slffOutEpNum2 == pMsg->data[0])
    {
        pAppCtxt->slffSlpRx2 = true;
        commitSize = Cy_USB_AppReadShortPacket(pAppCtxt, pAppCtxt->slffOutEpNum2, pMsg->data[1]);
        pAppCtxt->slffTxBufferCount2[pAppCtxt->slffNextTxBufIndex2] = commitSize;
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + pAppCtxt->slffOutEpNum2, CY_TRIGGER_TWO_CYCLES);
    }
    
} /* end of function. */

/**
 * \name Cy_USB_AppQueueRead
 * \brief Queue USBHS Write on the USB endpoint
 * \param pAppCtxt application layer context pointer.
 * \param endpNumber Endpoint number
 * \param pBuffer Data Buffer Pointer
 * \param dataSize DataSize to send on USB bus
 * \param index slavefifo  channel index
 * \retval None
 */
void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize, uint8_t index)
{
    cy_stc_app_endp_dma_set_t *dmaset_p=NULL;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL)) 
    {
        DBG_APP_ERR("QueueWrite Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0)) 
    {
        DBG_APP_ERR("QueueWrite Err1\r\n");
        return;
    }
    
    if (Cy_USBHS_App_QueueRead(dmaset_p, pBuffer, dataSize))
    {
        if( index == 1)
            pAppCtxt->slffReadInQueue1 = true;
        else if (index  == 2)
            pAppCtxt->slffReadInQueue2 = true;


        /* Set transfer size for the endpoint and clear NAK status to allow data to be received. */
        if (dataSize < dmaset_p->maxPktSize) 
        {
            /* We are trying to read out data that has already been received. Force EP NAK. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
        } 
        else 
        {
            /* Set the NAK bit so that the IP can see the bit transition from 1->0 after XFER_CNT is set. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
            Cy_USBD_UpdateXferCount(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, dataSize);
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, false);
        }
    }
} /* end of function */

/**
 * \name Cy_USB_AppInitDmaIntr
 * \brief Function to register an ISR for the DMA channel associated with an endpoint
 * \param endpNumber USB endpoint number
 * \param endpDirection Endpoint direction
 * \param userIsr ISR function pointer. Can be NULL if interrupt is to be disabled.
 * \retval None
 */
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
#if (!CY_CPU_CORTEX_M4)
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
		    intrCfg.intrPriority = 3;
        	intrCfg.intrSrc = NvicMux1_IRQn;
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
		else
		{
			intrCfg.intrPriority = 3;
        	intrCfg.intrSrc = NvicMux6_IRQn;
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
		}
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
}

/**
 * \name Cy_USB_AppClearDmaInterrupt
 * \brief Clear DMA Interrupt
 * \param pAppCtxt application layer context pointer.
 * \param endpNumber Endpoint number
 * \param endpDirection Endpoint direction
 * \retval None
 */
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                 uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
            (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
        } else  {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
        }
    }
} /* end of function. */

/**
 * \name Cy_Slff_AppHandleTxCompletion
 * \brief Slave FIFO transmit completion handler
 * \param pUsbApp application layer context pointer.
 * \param index Slave FIOF channel index number
 * \retval None
 */
void Cy_Slff_AppHandleTxCompletion (cy_stc_usb_app_ctxt_t *pUsbApp ,uint8_t index)
{
    cy_stc_hbdma_buff_status_t buffStat;
    cy_en_hbdma_mgr_status_t   status;

    if(index == 1)
    {
        pUsbApp->slffReadInQueue1 = false;
        if(pUsbApp->slffSlpRx1 == false)
        {
            pUsbApp->slffTxBufferCount1[pUsbApp->slffNextTxBufIndex1] = SLFF_TX_MAX_BUFFER_SIZE;
        }
        else
        {
            pUsbApp->slffSlpRx1 = false;
        }
        
        buffStat.size = SLFF_TX_MAX_BUFFER_SIZE;
        buffStat.pBuffer = pUsbApp->pSlffTxBuffer1[pUsbApp->slffNextTxBufIndex1];
        buffStat.count = pUsbApp->slffTxBufferCount1[pUsbApp->slffNextTxBufIndex1];
        buffStat.status = 0;
        
        pUsbApp->slffTxBufFullCount1++;
        pUsbApp->slffNextTxBufIndex1++;
        if(pUsbApp->slffNextTxBufIndex1 > 3)
        {
            pUsbApp->slffNextTxBufIndex1 = 0;
        }
        if(pUsbApp->slffTxBufFullCount1 < SLFF_TX_MAX_BUFFER_COUNT)
        {
            Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum1, pUsbApp->pSlffTxBuffer1[pUsbApp->slffNextTxBufIndex1], SLFF_TX_MAX_BUFFER_SIZE,index);
        }

        status = Cy_HBDma_Channel_CommitBuffer(pUsbApp->hbBulkOutChannel[0], &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("CommitBuffer failed with status=%x\r\n", status);
        }
        else
        {
            DBG_APP_TRACE("Index# %d Commmit Pass Size %x, Address %x, Count %d\r\n", index,buffStat.size,buffStat.pBuffer,buffStat.count);
        }
    }
    else if (index == 2)
    {
        pUsbApp->slffReadInQueue2 = false;
        if(pUsbApp->slffSlpRx2 == false)
        {
            pUsbApp->slffTxBufferCount2[pUsbApp->slffNextTxBufIndex2] = SLFF_TX_MAX_BUFFER_SIZE;
        }
        else
        {
            pUsbApp->slffSlpRx2 = false;
        }
        
        buffStat.size = SLFF_TX_MAX_BUFFER_SIZE;
        buffStat.pBuffer = pUsbApp->pSlffTxBuffer2[pUsbApp->slffNextTxBufIndex2];
        buffStat.count = pUsbApp->slffTxBufferCount2[pUsbApp->slffNextTxBufIndex2];
        buffStat.status = 0;
        
        pUsbApp->slffTxBufFullCount2++;
        pUsbApp->slffNextTxBufIndex2++;
        if(pUsbApp->slffNextTxBufIndex2 > 3)
        {
            pUsbApp->slffNextTxBufIndex2 = 0;
        }
        if(pUsbApp->slffTxBufFullCount2 < SLFF_TX_MAX_BUFFER_COUNT)
        {
            Cy_USB_AppQueueRead(pUsbApp, pUsbApp->slffOutEpNum2, pUsbApp->pSlffTxBuffer2[pUsbApp->slffNextTxBufIndex2], SLFF_TX_MAX_BUFFER_SIZE,index);
        }

        status = Cy_HBDma_Channel_CommitBuffer(pUsbApp->hbBulkOutChannel[1], &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("CommitBuffer failed with status=%x\r\n", status);
        }
        else
        {
            DBG_APP_TRACE("Index# %d Commmit Pass Size %x, Address %x, Count %d\r\n", index,buffStat.size,buffStat.pBuffer,buffStat.count);
        }

    }
}

/**
 * \name Cy_CheckStatus
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line Line number where error is seen
 * \param condition condition of failure
 * \param value error code
 * \param isBlocking blocking function
 * \retval None
 */
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/**
 * \name Cy_CheckStatusHandleFailure
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line LineNumber where error is seen
 * \param condition Line number where error is seen
 * \param value error code
 * \param isBlocking blocking function
 * \param failureHandler failure handler function
 * \retval None
 */
void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/**
 * \name Cy_USB_FailHandler
 * \brief Error Handler
 * \retval None
 */
void Cy_FailHandler(void)
{
    DBG_APP_ERR("Reset Done\r\n");
}

/* [] END OF FILE */
