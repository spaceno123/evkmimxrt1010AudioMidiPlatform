/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <stdlib.h>
/*${standard_header_anchor}*/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_midi.h"
#include "usb_device_audio.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "fsl_adapter_audio.h"
#include "composite.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#include "usb_phy.h"
#endif

#include "fsl_sai.h"
#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "fsl_codec_common.h"
#include "fsl_wm8960.h"
#include "fsl_codec_adapter.h"

#include "midi/MidiTask.h"
#include "audio/AudioTask.h"
#include "mylib/boardEx.h"

#include "debugMonitor/DebugMonitor.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_SAI_INSTANCE_INDEX (1U)
#define DEMO_SAI_TX_SOURCE      kDmaRequestMuxSai1Tx
#define DEMO_SAI_RX_SOURCE      kDmaRequestMuxSai1Rx
#define DEMO_SAI                SAI1
#define DEMO_DMA_INDEX          (0U)
#define DEMO_DMAMUX_INDEX       (0U)
#define DEMO_DMA_TX_CHANNEL     (0U)
#define DEMO_DMA_RX_CHANNEL     (1U)

/* Select Audio/Video PLL (786.432 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (3U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (15U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)

#define RXSAMPLEFLAME AUDIOSAMPLEFLAME	// <=6 for high speed
#define RXFORMATCHANNELS AUDIOCHANNELS
#define RXFORMATBITS  AUDIOFORMATBITS
#define RXFORMATSIZE  AUDIOFORMATSIZE
#define RXFIFOWATERMARK ((RXSAMPLEFLAME * RXFORMATCHANNELS / 2U) < ((uint32_t)FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) / 2U) ? \
		                 (RXSAMPLEFLAME * RXFORMATCHANNELS / 2U) : ((uint32_t)FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) / 2U))
#define TXSAMPLEFLAME (RXSAMPLEFLAME)
#define TXFORMATCHANNELS (RXFORMATCHANNELS)
#define TXFORMATBITS (RXFORMATBITS)
#define TXFORMATSIZE (RXFORMATSIZE)
#define TXFIFOWATERMARK ((TXSAMPLEFLAME * TXFORMATCHANNELS) < (uint8_t)(FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI)) ? \
		                 (FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) - (TXSAMPLEFLAME * TXFORMATCHANNELS)) : (uint8_t)(FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) - 1))
#define AUDIORECDMATEMPBUFFSIZE (RXSAMPLEFLAME * RXFORMATCHANNELS * RXFORMATSIZE)
#define AUDIOPLAYDMATEMPBUFFSIZE (TXSAMPLEFLAME * TXFORMATCHANNELS * TXFORMATSIZE)

#define MIDITASKPRIOLITY      (3)
#define APPTASKPRIOLITY       (4)	// USB device task
#define USBDEVICETASKPRIOLITY (5)	// USB function task (unused)
#define AUDIOTASKPRIOLITY     (8)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
#if !((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
void USB_DeviceHsPhyChirpIssueWorkaround(void);
void USB_DeviceDisconnected(void);
#endif
#endif
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
extern void USB_AudioCodecTask(void);
extern void USB_AudioSpeakerResetTask(void);
extern void USB_DeviceAudioSpeakerStatusReset(void);
extern void AUDIO_DMA_EDMA_Start();
extern void BOARD_Codec_Init();
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern usb_device_composite_struct_t g_composite;
extern uint8_t audioPlayDataBuff[AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT_NORMAL * AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME];
extern uint8_t audioRecDataBuff[AUDIO_RECORDER_DATA_WHOLE_BUFFER_COUNT_NORMAL * FS_ISO_IN_ENDP_PACKET_SIZE];
volatile bool g_ButtonPress = false;
HAL_AUDIO_HANDLE_DEFINE(audioTxHandle);
hal_audio_config_t audioTxConfig;
hal_audio_dma_config_t dmaTxConfig;
HAL_AUDIO_HANDLE_DEFINE(audioRxHandle);
hal_audio_config_t audioRxConfig;
hal_audio_dma_config_t dmaRxConfig;
hal_audio_ip_config_t ipTxConfig;
hal_audio_ip_config_t ipRxConfig;
hal_audio_dma_mux_config_t dmaMuxTxConfig;
hal_audio_dma_mux_config_t dmaMuxRxConfig;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t audioPlayDMATempBuff[AUDIOPLAYDMATEMPBUFFSIZE/*AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME*/];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t audioRecDMATempBuff[AUDIORECDMATEMPBUFFSIZE/*FS_ISO_IN_ENDP_PACKET_SIZE*/];
uint32_t masterClockHz = 0U;
codec_handle_t codecHandle;

wm8960_config_t wm8960Config = {
    .i2cConfig = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .route     = kWM8960_RoutePlaybackandRecord,
    .leftInputSource  = kWM8960_InputDifferentialMicInput3,
    .rightInputSource = kWM8960_InputDifferentialMicInput2,
    .playSource       = kWM8960_PlaySourceDAC,
    .slaveAddress     = WM8960_I2C_ADDR,
    .bus              = kWM8960_BusI2S,
    .format           = {.mclk_HZ    = 12288000U,
               .sampleRate = kWM8960_AudioSampleRate48KHz,
               .bitWidth   = RXFORMATBITS},//kWM8960_AudioBitWidth16bit},
    .master_slave     = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8960, .codecDevConfig = &wm8960Config};

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 768/1000)
 *                              = 786.432 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,   /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,    /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 768,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 1000, /* 30 bit denominator of fractional loop divider */
};
extern void WM8960_USB_Audio_Init(void *I2CBase, void *i2cHandle);
extern void WM8960_Config_Audio_Formats(uint32_t samplingRate);
extern uint32_t USB_AudioSpeakerBufferSpaceUsed(void);
extern void USB_DeviceCalculateFeedback(void);
/* Composite device structure. */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
usb_device_composite_struct_t g_composite;
extern usb_device_class_struct_t g_UsbDeviceMidiClass;
extern usb_device_class_struct_t g_UsbDeviceAudioClassRecorder;
extern usb_device_class_struct_t g_UsbDeviceAudioClassSpeaker;
extern volatile bool g_ButtonPress;
extern usb_device_composite_struct_t *g_UsbDeviceComposite;
extern usb_device_composite_struct_t *g_deviceAudioComposite;
extern uint8_t audioFeedBackBuffer[4];
extern hal_audio_config_t audioTxConfig;
extern hal_audio_config_t audioRxConfig;
extern HAL_AUDIO_HANDLE_DEFINE(audioTxHandle);
extern HAL_AUDIO_HANDLE_DEFINE(audioRxHandle);
/* USB device class information */
static usb_device_class_config_struct_t g_CompositeClassConfig[3] = {
		{
				USB_DeviceMidiCallback,
				(class_handle_t)NULL,
				&g_UsbDeviceMidiClass,
		},
		{
				USB_DeviceAudioCompositeCallback,
				(class_handle_t)NULL,
				&g_UsbDeviceAudioClassRecorder,
		},
		{
				USB_DeviceAudioCompositeCallback,
				(class_handle_t)NULL,
				&g_UsbDeviceAudioClassSpeaker,
		}

};

/* USB device class configuration information */
static usb_device_class_config_list_struct_t g_UsbDeviceCompositeConfigList = {
    g_CompositeClassConfig,
    USB_DeviceCallback,
    3,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

void BOARD_Codec_Init()
{
    CODEC_Init(&codecHandle, &boardCodecConfig);
}

static void txCallback(hal_audio_handle_t handle, hal_audio_status_t completionStatus, void *callbackParam)
{
    static volatile int flame_count = 0;
    uint32_t audioSpeakerPreReadDataCount = 0U;
    uint32_t preAudioSendCount            = 0U;
    hal_audio_transfer_t xfer             = {0};

//    TIMINGLOGI2STXI();
    if (0U != g_composite.audioUnified.startPlayFlag)
    {
    	if (USB_AudioSpeakerBufferSpaceUsed() < (g_composite.audioUnified.audioPlayTransferSize))
        {
    		void APPTaskWupAudioOutReset(void);

    		g_composite.audioUnified.startPlayFlag          = 0;
            g_composite.audioUnified.speakerDetachOrNoInput = 1;
        	flame_count = 0;

        	APPTaskWup(1 << AUDIOOUTRESETREQBIT);
        }
        else
        {
        	uint8_t *src = audioPlayDataBuff + g_composite.audioUnified.tdReadNumberPlay + flame_count;
        	uint8_t *dst = audioPlayDMATempBuff;
        	int n = TXSAMPLEFLAME * AUDIO_OUT_FORMAT_CHANNELS;

        	while (n--)
        	{
    		    *dst++ = 0;
#if AUDIO_OUT_FORMAT_SIZE == 3
    		    *dst++ = *src++;
#else
    		    *dat++ = 0;
#endif
        		*dst++ = *src++;
    		    *dst++ = *src++;
        	}
        	flame_count += TXSAMPLEFLAME * AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE;
        	if (flame_count >= g_composite.audioUnified.audioPlayTransferSize)
        	{
            	flame_count -= g_composite.audioUnified.audioPlayTransferSize;
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
                USB_DeviceCalculateFeedback();
#endif
                preAudioSendCount = g_composite.audioUnified.audioSendCount[0];
                g_composite.audioUnified.audioSendCount[0] += g_composite.audioUnified.audioPlayTransferSize;
                if (preAudioSendCount > g_composite.audioUnified.audioSendCount[0])
                {
                    g_composite.audioUnified.audioSendCount[1] += 1U;
                }
                g_composite.audioUnified.audioSendTimes++;
                g_composite.audioUnified.tdReadNumberPlay += g_composite.audioUnified.audioPlayTransferSize;
                if (g_composite.audioUnified.tdReadNumberPlay >= g_composite.audioUnified.audioPlayBufferSize)
                {
                    g_composite.audioUnified.tdReadNumberPlay = 0;
                }
                audioSpeakerPreReadDataCount = g_composite.audioUnified.audioSpeakerReadDataCount[0];
                g_composite.audioUnified.audioSpeakerReadDataCount[0] += g_composite.audioUnified.audioPlayTransferSize;
                if (audioSpeakerPreReadDataCount > g_composite.audioUnified.audioSpeakerReadDataCount[0])
                {
                    g_composite.audioUnified.audioSpeakerReadDataCount[1] += 1U;
                }
        	}
        }
    }
    else
    {
    	flame_count = 0;
    	memset(audioPlayDMATempBuff, 0, sizeof(audioPlayDMATempBuff));
    }
    AudioTask_txComplete(audioPlayDMATempBuff);
    xfer.dataSize = AUDIOPLAYDMATEMPBUFFSIZE;//AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME / (AUDIO_OUT_SAMPLING_RATE_KHZ / TXSAMPLEFLAME);
    xfer.data = audioPlayDMATempBuff;
    HAL_AudioTransferSendNonBlocking((hal_audio_handle_t)&audioTxHandle[0], &xfer);
}

static void rxCallback(hal_audio_handle_t handle, hal_audio_status_t completionStatus, void *callbackParam)
{
    static volatile int flame_count = 0;
    hal_audio_transfer_t xfer = {0};

//    TIMINGLOGI2SRXI();
    AudioTask_rxComplete(audioRecDMATempBuff);
    if (g_composite.audioUnified.startRec)
    {
    	uint8_t *dst = audioRecDataBuff + g_composite.audioUnified.tdWriteNumberRec + flame_count;
    	uint8_t *src = audioRecDMATempBuff;
    	int n = RXSAMPLEFLAME * AUDIO_IN_FORMAT_CHANNELS;

    	while (n--)
    	{
    		src++;
#if AUDIO_OUT_FORMAT_SIZE == 3
    		*dst++ = *src++;
#else
    		src++;
#endif
    		*dst++ = *src++;
    		*dst++ = *src++;
    	}
    	flame_count += RXSAMPLEFLAME * AUDIO_IN_FORMAT_CHANNELS * AUDIO_IN_FORMAT_SIZE;
        if (flame_count >= FS_ISO_IN_ENDP_PACKET_SIZE)
        {
        	flame_count -= FS_ISO_IN_ENDP_PACKET_SIZE;
            g_composite.audioUnified.tdWriteNumberRec += FS_ISO_IN_ENDP_PACKET_SIZE;
            if (g_composite.audioUnified.tdWriteNumberRec >=
                AUDIO_RECORDER_DATA_WHOLE_BUFFER_COUNT_NORMAL * FS_ISO_IN_ENDP_PACKET_SIZE)
            {
                g_composite.audioUnified.tdWriteNumberRec = 0;
            }
        }
    }
    else
    {
    	flame_count = 0;
    }
    xfer.dataSize = AUDIORECDMATEMPBUFFSIZE;//FS_ISO_IN_ENDP_PACKET_SIZE / (AUDIO_IN_SAMPLING_RATE_KHZ / RXSAMPLEFLAME);
    xfer.data     = audioRecDMATempBuff;
    HAL_AudioTransferReceiveNonBlocking(handle, &xfer);
}

void AUDIO_DMA_EDMA_Start()
{
    usb_echo("Init Audio SAI and CODEC\r\n");
    hal_audio_transfer_t xfer = {0};
    memset(audioPlayDMATempBuff, 0, AUDIOPLAYDMATEMPBUFFSIZE/*AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME*/);
    memset(audioRecDMATempBuff, 0, AUDIORECDMATEMPBUFFSIZE/*FS_ISO_IN_ENDP_PACKET_SIZE*/);
    for (int i = 0; i < FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI); ++i)
    {
    	DEMO_SAI->TDR[0] = 0;
    }
    TIMINGLOGI2SINI();
    xfer.dataSize = AUDIOPLAYDMATEMPBUFFSIZE;
    xfer.data     = audioPlayDMATempBuff;
    HAL_AudioTxInstallCallback((hal_audio_handle_t)&audioTxHandle[0], txCallback, NULL);
    HAL_AudioTransferSendNonBlocking((hal_audio_handle_t)&audioTxHandle[0], &xfer);
    xfer.dataSize = AUDIORECDMATEMPBUFFSIZE;
    xfer.data     = audioRecDMATempBuff;
    HAL_AudioRxInstallCallback((hal_audio_handle_t)&audioRxHandle[0], rxCallback, NULL);
    HAL_AudioTransferReceiveNonBlocking((hal_audio_handle_t)&audioRxHandle[0], &xfer);
}

void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
}

void USB_DeviceClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber                  = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
    USB_DeviceEhciTaskFunction(deviceHandle);
}
#endif

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle		  The USB device handle.
 * @param event 		  The USB device event type.
 * @param param 		  The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;
    uint8_t count      = 0U;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            for (count = 0U; count < USB_DEVICE_INTERFACE_COUNT; count++)
            {
                g_composite.currentInterfaceAlternateSetting[count] = 0U;
            }
            /* reset audio speaker status to be the initialized status */
            USB_DeviceAudioSpeakerStatusReset();
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
            /* reset the the last feedback value */
            g_composite.audioUnified.lastFeedbackValue             = 0U;
#endif
            g_composite.attach               = 0U;
            g_composite.currentConfiguration = 0U;
            error                            = kStatus_USB_Success;

#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
#if !((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
            /* The work-around is used to fix the HS device Chirping issue.
             * Please refer to the implementation for the detail information.
             */
            USB_DeviceHsPhyChirpIssueWorkaround();
#endif
#endif

#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_composite.speed))
            {
                USB_DeviceSetSpeed(handle, g_composite.speed);
            }
            if (USB_SPEED_HIGH == g_composite.speed)
            {
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
                g_composite.audioUnified.currentStreamOutMaxPacketSize = (HS_ISO_OUT_ENDP_PACKET_SIZE);
#else
                g_composite.audioUnified.currentStreamOutMaxPacketSize =
                    (HS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE);
                g_composite.audioUnified.currentFeedbackMaxPacketSize = HS_ISO_FEEDBACK_ENDP_PACKET_SIZE;
#endif /* USB_DEVICE_AUDIO_USE_SYNC_MODE */
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
                /* high speed and audio 2.0, audio play interval is set by HS EP packet size */
                g_composite.audioUnified.audioPlayTransferSize = HS_ISO_OUT_ENDP_PACKET_SIZE;
                /* use short play buffer size, only use two elements */
                g_composite.audioUnified.audioPlayBufferSize =
                    AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME * AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT;
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
                AUDIO_UPDATE_FEEDBACK_DATA(audioFeedBackBuffer, AUDIO_SAMPLING_RATE_TO_16_16_SPECIFIC);
#else
                AUDIO_UPDATE_FEEDBACK_DATA(audioFeedBackBuffer, AUDIO_SAMPLING_RATE_TO_16_16);
#endif
#endif
#else
                /* high speed and audio 1.0, audio play interval is 1 ms using this play size */
                g_composite.audioUnified.audioPlayTransferSize = AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
                /* use the whole play buffer size */
                g_composite.audioUnified.audioPlayBufferSize =
                    AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT_NORMAL * AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
                AUDIO_UPDATE_FEEDBACK_DATA(audioFeedBackBuffer, AUDIO_SAMPLING_RATE_TO_10_14);
#endif
#endif /* USB_DEVICE_CONFIG_AUDIO_CLASS_2_0 */
                g_deviceAudioComposite->audioUnified.speed = USB_SPEED_HIGH;
            }
            else
            {
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
                g_composite.audioUnified.currentStreamOutMaxPacketSize = (FS_ISO_OUT_ENDP_PACKET_SIZE);
#else
                g_composite.audioUnified.currentStreamOutMaxPacketSize =
                    (FS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE);
                g_composite.audioUnified.currentFeedbackMaxPacketSize = FS_ISO_FEEDBACK_ENDP_PACKET_SIZE;
                AUDIO_UPDATE_FEEDBACK_DATA(audioFeedBackBuffer, AUDIO_SAMPLING_RATE_TO_10_14);
#endif
                /* full speed, audio play interval is 1 ms using this play size */
                g_composite.audioUnified.audioPlayTransferSize = AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
                /* use the whole play buffer size */
                g_composite.audioUnified.audioPlayBufferSize =
                    AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT_NORMAL * AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
            }
#else
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
            g_composite.audioUnified.currentStreamOutMaxPacketSize = (FS_ISO_OUT_ENDP_PACKET_SIZE);
#else
            g_composite.audioUnified.currentStreamOutMaxPacketSize =
                (FS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE);
            g_composite.audioUnified.currentFeedbackMaxPacketSize = FS_ISO_FEEDBACK_ENDP_PACKET_SIZE;
            AUDIO_UPDATE_FEEDBACK_DATA(audioFeedBackBuffer, AUDIO_SAMPLING_RATE_TO_10_14);
            /* reset the the last feedback value */
            g_deviceAudioComposite->audioUnified.lastFeedbackValue = 0U;
#endif
            /* full speed, audio play interval is 1 ms using this play size */
            g_composite.audioUnified.audioPlayTransferSize = AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
            /* use the whole play buffer size */
            g_composite.audioUnified.audioPlayBufferSize =
                AUDIO_SPEAKER_DATA_WHOLE_BUFFER_COUNT_NORMAL * AUDIO_PLAY_BUFFER_SIZE_ONE_FRAME;
#endif /* USB_DEVICE_CONFIG_EHCI, USB_DEVICE_CONFIG_LPCIP3511HS */
        }
        break;
#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
        case kUSB_DeviceEventDetach:
        {
#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
#if !((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
            USB_DeviceDisconnected();
#endif
#endif
            error = kStatus_USB_Success;
        }
        break;
#endif
        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                g_composite.attach               = 0U;
                g_composite.currentConfiguration = 0U;
                error                            = kStatus_USB_Success;
            }
            else if (USB_COMPOSITE_CONFIGURE_INDEX == (*temp8))
            {
                g_composite.attach               = 1U;
                g_composite.currentConfiguration = *temp8;
                USB_DeviceAudioCompositeSetConfigure(g_composite.audioUnified.audioSpeakerHandle, *temp8);
                USB_DeviceAudioCompositeSetConfigure(g_composite.audioUnified.audioRecorderHandle, *temp8);
                USB_DeviceMidiSetConfigure(g_composite.midiStream.midiHandle, *temp8);
                error = kStatus_USB_Success;
            }
            else
            {
            }
            break;
        case kUSB_DeviceEventSetInterface:

            if (g_composite.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);

                switch (interface)
                {
                    case USB_AUDIO_CONTROL_INTERFACE_INDEX:
                        if (alternateSetting < USB_AUDIO_CONTROL_INTERFACE_ALTERNATE_COUNT)
                        {
                            error = kStatus_USB_Success;
                        }
                        break;
                    case USB_AUDIO_RECORDER_STREAM_INTERFACE_INDEX:
                        if (alternateSetting < USB_AUDIO_RECORDER_STREAM_INTERFACE_ALTERNATE_COUNT)
                        {
                            error = USB_DeviceAudioRecorderSetInterface(g_composite.audioUnified.audioRecorderHandle,
                                                                        interface, alternateSetting);
                        }
                        break;
                    case USB_AUDIO_SPEAKER_STREAM_INTERFACE_INDEX:
                        if (alternateSetting < USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_COUNT)
                        {
                            error = USB_DeviceAudioSpeakerSetInterface(g_composite.audioUnified.audioSpeakerHandle,
                                                                       interface, alternateSetting);
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
                            if (USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_1 == alternateSetting)
                            {
                                g_composite.audioUnified.stopDataLengthAudioAdjust = 0U;
                            }
                            else if (USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_0 == alternateSetting)
                            {
                                g_composite.audioUnified.stopDataLengthAudioAdjust = 1U;
                            }
#else
                            /* usb host stops the speaker, so there is no need for feedback */
                            if ((1U == g_composite.audioUnified.startPlayFlag) &&
                                (USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_0 == alternateSetting))
                            {
                                g_composite.audioUnified.stopFeedbackUpdate = 1U;
                            }

                            /* usb host start the speaker, discard the feedback for AUDIO_SPEAKER_FEEDBACK_DISCARD_COUNT
                             * times */
                            if (USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_1 == alternateSetting)
                            {
                                g_composite.audioUnified.stopFeedbackUpdate              = 0U;
                                g_deviceAudioComposite->audioUnified.feedbackDiscardFlag = 1U;
                                g_deviceAudioComposite->audioUnified.feedbackDiscardTimes =
                                    AUDIO_SPEAKER_FEEDBACK_DISCARD_COUNT;
                            }
#endif
                        }
                        break;
                    case USB_MIDI_INTERFACE_INDEX:
                        if (alternateSetting < USB_MIDI_INTERFACE_ALTERNATE_COUNT)
                        {
                            error = kStatus_USB_Success;
                        }
                    	break;
                    default:
                        break;
                }

                if (kStatus_USB_Success == error)
                {
                    g_composite.currentInterfaceAlternateSetting[interface] = alternateSetting;
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                *temp8 = g_composite.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_DEVICE_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | g_composite.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
        case kUSB_DeviceEventGetDeviceQualifierDescriptor:
            if (param)
            {
                /* Get device descriptor request */
                error = USB_DeviceGetDeviceQualifierDescriptor(
                    handle, (usb_device_get_device_qualifier_descriptor_struct_t *)param);
            }
            break;
#endif
        default:
            break;
    }

    return error;
}

/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */
void APPInit(void)
{
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    g_composite.speed                            = USB_SPEED_FULL;
    g_composite.attach                           = 0U;
    g_composite.audioUnified.audioSpeakerHandle  = (class_handle_t)NULL;
    g_composite.audioUnified.audioRecorderHandle = (class_handle_t)NULL;
    g_composite.midiStream.midiHandle            = (class_handle_t)NULL;
    g_composite.deviceHandle                     = NULL;

    if (kStatus_USB_Success !=
        USB_DeviceClassInit(CONTROLLER_ID, &g_UsbDeviceCompositeConfigList, &g_composite.deviceHandle))
    {
        usb_echo("Basic Platform init failed\r\n");
        return;
    }
    else
    {
        usb_echo("Basic Platform\r\n");

        g_composite.midiStream.midiHandle            = g_UsbDeviceCompositeConfigList.config[0].classHandle;
        g_composite.audioUnified.audioRecorderHandle = g_UsbDeviceCompositeConfigList.config[1].classHandle;
        g_composite.audioUnified.audioSpeakerHandle  = g_UsbDeviceCompositeConfigList.config[2].classHandle;

        USB_DeviceAudioCompositeInit(&g_composite);
        USB_DeviceMidiStreamInit(&g_composite);
    }

    /*Initialize audio interface and codec.*/
    HAL_AudioTxInit((hal_audio_handle_t)audioTxHandle, &audioTxConfig);
    HAL_AudioRxInit((hal_audio_handle_t)audioRxHandle, &audioRxConfig);
    BOARD_Codec_Init();
    AUDIO_DMA_EDMA_Start();

    USB_DeviceIsrEnable();

    /*Add one delay here to make the DP pull down long enough to allow host to detect the previous disconnection.*/
    SDK_DelayAtLeastUs(5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    USB_DeviceRun(g_composite.deviceHandle);
}

/*!
 * @brief USB task function.
 *
 * This function runs the task for USB device.
 *
 * @return None.
 */
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTask(void *handle)
{
    while (1U)
    {
        USB_DeviceTaskFn(handle);
    }
}
#endif

/*!
 * @brief Application task function.
 *
 * This function runs the task for application.
 *
 * @return None.
 */
void APPTask(void *handle)
{
    APPInit();

#if USB_DEVICE_CONFIG_USE_TASK
    if (g_composite.deviceHandle)
    {
        if (xTaskCreate(USB_DeviceTask,                  /* pointer to the task */
                        (char const *)"usb device task", /* task name for kernel awareness debugging */
                        5000L / sizeof(portSTACK_TYPE),  /* task stack size */
                        g_composite.deviceHandle,        /* optional task startup argument */
						USBDEVICETASKPRIOLITY,           /* initial priority */
                        &g_composite.deviceTaskHandle    /* optional task handle to create */
                        ) != pdPASS)
        {
            usb_echo("usb device task create failed!\r\n");
            return;
        }
    }
#endif

    while (1)
    {
    	uint32_t req = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    	if (req & ((1 << MIDITXREQBIT)|(1 << MIDIRXREQBIT)|(1 << MIDITXREMAINREQBIT)))
    	{
        	USB_DeviceMidiStreamTask(req);
    	}
    	if (req & (1 << AUDIOOUTRESETREQBIT))
    	{
            USB_AudioSpeakerResetTask();
    	}
    }
}

void APPTaskWup(uint32_t bitPattern)
{
	if (xPortIsInsideInterrupt())
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(g_composite.applicationTaskHandle, bitPattern, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		xTaskNotify(g_composite.applicationTaskHandle, bitPattern, eSetBits);
	}

	return;
}

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_RelocateVectorTableToRam();

	BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    CLOCK_InitAudioPll(&audioPllConfig);
    BOARD_InitDebugConsole();

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);

    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);

    dmaMuxTxConfig.dmaMuxConfig.dmaMuxInstance   = DEMO_DMAMUX_INDEX;
    dmaMuxTxConfig.dmaMuxConfig.dmaRequestSource = DEMO_SAI_TX_SOURCE;
    dmaTxConfig.dmaMuxConfig                     = &dmaMuxTxConfig;
    dmaTxConfig.instance                         = DEMO_DMA_INDEX;
    dmaTxConfig.channel                          = DEMO_DMA_TX_CHANNEL;
    dmaTxConfig.priority                         = kHAL_AudioDmaChannelPriorityDefault;
    dmaTxConfig.dmaChannelMuxConfig              = NULL;
    ipTxConfig.sai.lineMask                      = 1U << 0U;
    ipTxConfig.sai.syncMode                      = kHAL_AudioSaiModeAsync;
    audioTxConfig.dmaConfig                      = &dmaTxConfig;
    audioTxConfig.ipConfig                       = &ipTxConfig;
    audioTxConfig.instance                       = DEMO_SAI_INSTANCE_INDEX;
    audioTxConfig.srcClock_Hz                    = DEMO_SAI_CLK_FREQ;
    audioTxConfig.sampleRate_Hz                  = (uint32_t)kHAL_AudioSampleRate48KHz;
    audioTxConfig.masterSlave                    = kHAL_AudioMaster;
    audioTxConfig.bclkPolarity                   = kHAL_AudioSampleOnRisingEdge;
    audioTxConfig.frameSyncWidth                 = kHAL_AudioFrameSyncWidthHalfFrame;
    audioTxConfig.frameSyncPolarity              = kHAL_AudioBeginAtFallingEdge;
    audioTxConfig.dataFormat                     = kHAL_AudioDataFormatI2sClassic;
    audioTxConfig.fifoWatermark                  = TXFIFOWATERMARK;//(uint8_t)(FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) - 1);
    audioTxConfig.bitWidth                       = TXFORMATBITS;//(uint8_t)kHAL_AudioWordWidth16bits;
    audioTxConfig.lineChannels                   = kHAL_AudioStereo;

    dmaMuxRxConfig.dmaMuxConfig.dmaMuxInstance   = DEMO_DMAMUX_INDEX;
    dmaMuxRxConfig.dmaMuxConfig.dmaRequestSource = DEMO_SAI_RX_SOURCE;
    dmaRxConfig.dmaMuxConfig                     = &dmaMuxRxConfig;
    dmaRxConfig.instance                         = DEMO_DMA_INDEX;
    dmaRxConfig.channel                          = DEMO_DMA_RX_CHANNEL;
    dmaRxConfig.priority                         = kHAL_AudioDmaChannelPriorityDefault;
    dmaRxConfig.dmaChannelMuxConfig              = NULL;
    ipRxConfig.sai.lineMask                      = 1U << 0U;
    ipRxConfig.sai.syncMode                      = kHAL_AudioSaiModeSync;
    audioRxConfig.dmaConfig                      = &dmaRxConfig;
    audioRxConfig.ipConfig                       = &ipRxConfig;
    audioRxConfig.instance                       = DEMO_SAI_INSTANCE_INDEX;
    audioRxConfig.srcClock_Hz                    = DEMO_SAI_CLK_FREQ;
    audioRxConfig.sampleRate_Hz                  = (uint32_t)kHAL_AudioSampleRate48KHz;
    audioRxConfig.masterSlave                    = kHAL_AudioMaster;
    audioRxConfig.bclkPolarity                   = kHAL_AudioSampleOnRisingEdge;
    audioRxConfig.frameSyncWidth                 = kHAL_AudioFrameSyncWidthHalfFrame;
    audioRxConfig.frameSyncPolarity              = kHAL_AudioBeginAtFallingEdge;
    audioRxConfig.dataFormat                     = kHAL_AudioDataFormatI2sClassic;
    audioRxConfig.fifoWatermark                  = RXFIFOWATERMARK;//(uint8_t)((uint32_t)FSL_FEATURE_SAI_FIFO_COUNTn(DEMO_SAI) / 2U);
    audioRxConfig.bitWidth                       = RXFORMATBITS;//(uint8_t)kHAL_AudioWordWidth16bits;
    audioRxConfig.lineChannels                   = kHAL_AudioStereo;

    if (xTaskCreate(APPTask,                           /* pointer to the task */
                    (char const *)"usb device task",   /* task name for kernel awareness debugging */
                    1000L/*5000L*/ / sizeof(portSTACK_TYPE),    /* task stack size */
                    &g_composite,                      /* optional task startup argument */
					APPTASKPRIOLITY,                   /* initial priority */
                    &g_composite.applicationTaskHandle /* optional task handle to create */
                    ) != pdPASS)
    {
        usb_echo("app task create failed!\r\n");
#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
        return 1;
#else
        return;
#endif
    }

    if (xTaskCreate(MidiTask,
    				(char const *)"midi task",
					1000L / sizeof(portSTACK_TYPE),
					NULL,
					MIDITASKPRIOLITY,
					NULL
    				) != pdPASS)
    {
        usb_echo("midi task create failed!\r\n");
#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
        return 1;
#else
        return;
#endif
    }

    if (xTaskCreate(AudioTask,
    				(char const *)"audio task",
					1000L / sizeof(portSTACK_TYPE),
					NULL,
					AUDIOTASKPRIOLITY,
					NULL
    				) != pdPASS)
    {
        usb_echo("audio task create failed!\r\n");
#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
        return 1;
#else
        return;
#endif
    }

    vTaskStartScheduler();

#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
    return 1;
#endif
}
