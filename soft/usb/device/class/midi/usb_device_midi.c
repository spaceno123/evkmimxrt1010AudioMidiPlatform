/*
 * usb_device_midi.c
 *
 *  Created on: 2024/08/29
 *      Author: M.Akino
 */
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"

#if ((defined(USB_DEVICE_CONFIG_MIDI)) && (USB_DEVICE_CONFIG_MIDI > 0U))
#include "usb_device_midi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceMidiAllocateHandle(usb_device_midi_struct_t **handle);
static usb_status_t USB_DeviceMidiFreeHandle(usb_device_midi_struct_t *handle);
static usb_status_t USB_DeviceMidiBulkIn(usb_device_handle handle,
                                         usb_device_endpoint_callback_message_struct_t *message,
                                         void *callbackParam);
static usb_status_t USB_DeviceMidiBulkOut(usb_device_handle handle,
                                          usb_device_endpoint_callback_message_struct_t *message,
                                          void *callbackParam);
static usb_status_t USB_DeviceMidiEndpointsInit(usb_device_midi_struct_t *midiHandle);
static usb_status_t USB_DeviceMidiEndpointsDeinit(usb_device_midi_struct_t *midiHandle);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_midi_struct_t
    s_UsbDeviceMidiHandle[USB_DEVICE_CONFIG_MIDI];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Allocate a device midi class handle.
 *
 * This function allocates a device midi class handle.
 *
 * @param handle          It is out parameter, is used to return pointer of the device midi class handle to the caller.
 *
 * @retval kStatus_USB_Success              Get a device midi class handle successfully.
 * @retval kStatus_USB_Busy                 Cannot allocate a device midi class handle.
 */
static usb_status_t USB_DeviceMidiAllocateHandle(usb_device_midi_struct_t **handle)
{
    uint32_t count;
    for (count = 0U; count < USB_DEVICE_CONFIG_MIDI; count++)
    {
        if (NULL == s_UsbDeviceMidiHandle[count].handle)
        {
            *handle = &s_UsbDeviceMidiHandle[count];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}

/*!
 * @brief Free a device midi class handle.
 *
 * This function frees a device midi class handle.
 *
 * @param handle          The device midi class handle.
 *
 * @retval kStatus_USB_Success              Free device midi class handle successfully.
 */
static usb_status_t USB_DeviceMidiFreeHandle(usb_device_midi_struct_t *handle)
{
    handle->handle          = NULL;
    handle->configStruct    = (usb_device_class_config_struct_t *)NULL;
    handle->configuration   = 0U;
    handle->alternate       = 0U;
    return kStatus_USB_Success;
}

/*!
 * @brief Responds to the bulk in endpoint event.
 *
 * This function responds to the bulk in endpoint event.
 *
 * @param handle The device handle of the MIDI device.
 * @param message The pointer to the message of the endpoint callback.
 * @param callbackParam The pointer to the parameter of the callback.
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMidiBulkIn(usb_device_handle handle,
                                         usb_device_endpoint_callback_message_struct_t *message,
                                         void *callbackParam)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t status = kStatus_USB_Error;
    midiHandle          = (usb_device_midi_struct_t *)callbackParam;

    if (NULL == midiHandle)
    {
        return kStatus_USB_InvalidHandle;
    }

    midiHandle->inPipeBusy = 0U;

    if ((NULL != midiHandle->configStruct) && (NULL != midiHandle->configStruct->classCallback))
    {
        /*classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = midiHandle->configStruct->classCallback((class_handle_t)midiHandle,
                                                         kUSB_DeviceMidiEventSendResponse, message);
    }
    return status;
}

/*!
 * @brief Responds to the bulk out endpoint event.
 *
 * This function responds to the bulk out endpoint event.
 *
 * @param handle The device handle of the MIDI device.
 * @param message The pointer to the message of the endpoint callback.
 * @param callbackParam The pointer to the parameter of the callback.
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMidiBulkOut(usb_device_handle handle,
                                          usb_device_endpoint_callback_message_struct_t *message,
                                          void *callbackParam)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t status = kStatus_USB_Error;
    midiHandle          = (usb_device_midi_struct_t *)callbackParam;

    if (NULL == midiHandle)
    {
        return kStatus_USB_InvalidHandle;
    }

    midiHandle->outPipeBusy = 0U;

    if ((NULL != midiHandle->configStruct) && (NULL != midiHandle->configStruct->classCallback))
    {
        /*classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = midiHandle->configStruct->classCallback((class_handle_t)midiHandle,
                                                         kUSB_DeviceMidiEventRecvResponse, message);
    }
    return status;
}

/*!
 * @brief Initialize the endpoints of the midi class.
 *
 * This callback function is used to initialize the endpoints of the midi class.
 *
 * @param midiHandle         The device midi class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMidiEndpointsInit(usb_device_midi_struct_t *midiHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t status                      = kStatus_USB_Error;
    uint32_t count;
    uint32_t index;

    /* Check the configuration is valid or not. */
    if (0U == midiHandle->configuration)
    {
        return status;
    }

    if (midiHandle->configuration > midiHandle->configStruct->classInfomation->configurations)
    {
        return status;
    }

    /* Get the interface list of the new configuration. */
    if (NULL == midiHandle->configStruct->classInfomation->interfaceList)
    {
        return status;
    }
    interfaceList = &midiHandle->configStruct->classInfomation->interfaceList[midiHandle->configuration - 1U];

    /* Find interface by using the alternate setting of the interface. */
    for (count = 0U; count < interfaceList->count; count++)
    {
        if (USB_DEVICE_CONFIG_MIDI_CLASS_CODE == interfaceList->interfaces[count].classCode)
        {
            for (index = 0U; index < interfaceList->interfaces[count].count; index++)
            {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == midiHandle->alternate)
                {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            midiHandle->interfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface)
    {
        /* Return error if the interface is not found. */
        return status;
    }

    /* Keep new interface handle. */
    midiHandle->interfaceHandle = interface;

    /* Initialize the endpoints of the new interface. */
    for (count = 0U; count < interface->endpointList.count; count++)
    {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0U;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                       USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
        {
            epCallback.callbackFn        = USB_DeviceMidiBulkIn;
            midiHandle->inPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
            midiHandle->inPipeStall      = 0U;
            midiHandle->inPipeDataLen    = 0U;
        }
        else
        {
            epCallback.callbackFn         = USB_DeviceMidiBulkOut;
            midiHandle->outPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
            midiHandle->outPipeStall      = 0U;
            midiHandle->outPipeDataLen    = 0U;
        }
        epCallback.callbackParam = midiHandle;

        status = USB_DeviceInitEndpoint(midiHandle->handle, &epInitStruct, &epCallback);
    }
    return status;
}

/*!
 * @brief De-initialize the endpoints of the midi class.
 *
 * This callback function is used to de-initialize the endpoints of the midi class.
 *
 * @param midiHandle         The device midi class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMidiEndpointsDeinit(usb_device_midi_struct_t *midiHandle)
{
    usb_status_t status = kStatus_USB_Error;
    uint32_t count;

    if (NULL == midiHandle->interfaceHandle)
    {
        return status;
    }
    /* De-initialize all endpoints of the interface */
    for (count = 0U; count < midiHandle->interfaceHandle->endpointList.count; count++)
    {
        status = USB_DeviceDeinitEndpoint(midiHandle->handle,
                                          midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress);
    }
    midiHandle->interfaceHandle = NULL;
    return status;
}

/*!
 * @brief Handle the event passed to the midi class.
 *
 * This function handles the event passed to the midi class.
 *
 * @param handle          The midi class handle, got from the usb_device_class_config_struct_t::classHandle.
 * @param event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param param           The param type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe will be stalled by the caller.
 */
usb_status_t USB_DeviceMidiEvent(void *handle, uint32_t event, void *param)
{
    usb_device_midi_struct_t *midiHandle;
//?    usb_device_midi_report_struct_t report;
    usb_status_t error = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint32_t count;
    uint8_t *temp8;
    uint8_t alternate;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;

    if ((NULL == param) || (NULL == handle))
    {
        return kStatus_USB_InvalidHandle;
    }
//?    report.reportBuffer = (uint8_t *)NULL;
//?    report.reportLength = 0U;

    /* Get the midi class handle. */
    midiHandle = (usb_device_midi_struct_t *)handle;

    switch (eventCode)
    {
        case kUSB_DeviceClassEventDeviceReset:
            /* Bus reset, clear the configuration. */
            midiHandle->configuration   = 0U;
            midiHandle->inPipeBusy      = 0U;
            midiHandle->outPipeBusy     = 0U;
            midiHandle->interfaceHandle = NULL;
            error                       = kStatus_USB_Success;
            break;
        case kUSB_DeviceClassEventSetConfiguration:
            /* Get the new configuration. */
            temp8 = ((uint8_t *)param);
            if (NULL == midiHandle->configStruct)
            {
                break;
            }
            if (*temp8 == midiHandle->configuration)
            {
                error = kStatus_USB_Success;
                break;
            }

            /* De-initialize the endpoints when current configuration is none zero. */
            if (0U != midiHandle->configuration)
            {
                error = USB_DeviceMidiEndpointsDeinit(midiHandle);
            }
            /* Save new configuration. */
            midiHandle->configuration = *temp8;
            /* Clear the alternate setting value. */
            midiHandle->alternate = 0U;

            /* Initialize the endpoints of the new current configuration by using the alternate setting 0. */
            error = USB_DeviceMidiEndpointsInit(midiHandle);
            break;
        case kUSB_DeviceClassEventSetInterface:
            if (NULL == midiHandle->configStruct)
            {
                break;
            }
            /* Get the new alternate setting of the interface */
            interfaceAlternate = *((uint16_t *)param);
            /* Get the alternate setting value */
            alternate = (uint8_t)(interfaceAlternate & 0xFFU);

            /* Whether the interface belongs to the class. */
            if (midiHandle->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8U)))
            {
                break;
            }
            /* Only handle new alternate setting. */
            if (alternate == midiHandle->alternate)
            {
                error = kStatus_USB_Success;
                break;
            }
            /* De-initialize old endpoints */
            error                 = USB_DeviceMidiEndpointsDeinit(midiHandle);
            midiHandle->alternate = alternate;
            /* Initialize new endpoints */
            error = USB_DeviceMidiEndpointsInit(midiHandle);
            break;
        case kUSB_DeviceClassEventSetEndpointHalt:
            if ((NULL == midiHandle->configStruct) || (NULL == midiHandle->interfaceHandle))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            for (count = 0U; count < midiHandle->interfaceHandle->endpointList.count; count++)
            {
                if (*temp8 == midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
                {
                    /* Only stall the endpoint belongs to the class */
                    if (USB_IN == ((midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
                    {
                        midiHandle->inPipeStall = 1U;
                    }
                    else
                    {
                        midiHandle->outPipeStall = 1U;
                    }
                    error = USB_DeviceStallEndpoint(midiHandle->handle, *temp8);
                }
            }
            break;
        case kUSB_DeviceClassEventClearEndpointHalt:
            if ((NULL == midiHandle->configStruct) || (NULL == midiHandle->interfaceHandle))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            for (count = 0U; count < midiHandle->interfaceHandle->endpointList.count; count++)
            {
                if (*temp8 == midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
                {
                    /* Only un-stall the endpoint belongs to the class */
                    error = USB_DeviceUnstallEndpoint(midiHandle->handle, *temp8);
                    if (USB_IN == (((*temp8) & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                                   USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
                    {
                        if (0U != midiHandle->inPipeStall)
                        {
                            midiHandle->inPipeStall = 0U;
                            if ((uint8_t *)USB_INVALID_TRANSFER_BUFFER != midiHandle->inPipeDataBuffer)
                            {
                                error = USB_DeviceSendRequest(
                                    midiHandle->handle,
                                    (midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK),
                                    midiHandle->inPipeDataBuffer, midiHandle->inPipeDataLen);
                                if (kStatus_USB_Success != error)
                                {
                                    usb_device_endpoint_callback_message_struct_t endpointCallbackMessage;
                                    endpointCallbackMessage.buffer  = midiHandle->inPipeDataBuffer;
                                    endpointCallbackMessage.length  = midiHandle->inPipeDataLen;
                                    endpointCallbackMessage.isSetup = 0U;
#if (defined(USB_DEVICE_CONFIG_RETURN_VALUE_CHECK) && (USB_DEVICE_CONFIG_RETURN_VALUE_CHECK > 0U))
                                    if (kStatus_USB_Success !=
                                        USB_DeviceMidiBulkIn(midiHandle->handle, (void *)&endpointCallbackMessage,
                                                             handle))
                                    {
                                        return kStatus_USB_Error;
                                    }
#else
                                    (void)USB_DeviceMidiBulkIn(midiHandle->handle, (void *)&endpointCallbackMessage,
                                                               handle);
#endif
                                }
                                midiHandle->inPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
                                midiHandle->inPipeDataLen    = 0U;
                            }
                        }
                    }
                    else
                    {
                        if (0U != midiHandle->outPipeStall)
                        {
                            midiHandle->outPipeStall = 0U;
                            if ((uint8_t *)USB_INVALID_TRANSFER_BUFFER != midiHandle->outPipeDataBuffer)
                            {
                                error = USB_DeviceRecvRequest(
                                    midiHandle->handle,
                                    (midiHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress &
                                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK),
                                    midiHandle->outPipeDataBuffer, midiHandle->outPipeDataLen);
                                if (kStatus_USB_Success != error)
                                {
                                    usb_device_endpoint_callback_message_struct_t endpointCallbackMessage;
                                    endpointCallbackMessage.buffer  = midiHandle->outPipeDataBuffer;
                                    endpointCallbackMessage.length  = midiHandle->outPipeDataLen;
                                    endpointCallbackMessage.isSetup = 0U;
#if (defined(USB_DEVICE_CONFIG_RETURN_VALUE_CHECK) && (USB_DEVICE_CONFIG_RETURN_VALUE_CHECK > 0U))
                                    if (kStatus_USB_Success !=
                                        USB_DeviceMidiBulkOut(midiHandle->handle, (void *)&endpointCallbackMessage,
                                                              handle))
                                    {
                                        return kStatus_USB_Error;
                                    }
#else
                                    (void)USB_DeviceMidiBulkOut(midiHandle->handle, (void *)&endpointCallbackMessage,
                                                                handle);
#endif
                                }
                                midiHandle->outPipeDataBuffer = (uint8_t *)USB_INVALID_TRANSFER_BUFFER;
                                midiHandle->outPipeDataLen    = 0U;
                            }
                        }
                    }
                }
            }
            break;
        case kUSB_DeviceClassEventClassRequest:
        {
            /* Handle the midi class specific request. */
            usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;

            if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) !=
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                break;
            }

            if ((controlRequest->setup->wIndex & 0xFFU) != midiHandle->interfaceNumber)
            {
                break;
            }

            error = kStatus_USB_InvalidRequest;
            switch (controlRequest->setup->bRequest)
            {
            default:
                /* no action, return kStatus_USB_InvalidRequest */
                break;
            }
        }
        break;
        default:
            /*no action*/
            break;
    }
    return error;
}

/*!
 * @brief Initialize the midi class.
 *
 * This function is used to initialize the midi class.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param config          The class configuration information.
 * @param handle          It is out parameter, is used to return pointer of the midi class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMidiInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t error;

    /* Allocate a midi class handle. */
    error = USB_DeviceMidiAllocateHandle(&midiHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    /* Get the device handle according to the controller id. */
    error = USB_DeviceClassGetDeviceHandle(controllerId, &midiHandle->handle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    if (NULL == midiHandle->handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* Save the configuration of the class. */
    midiHandle->configStruct = config;
    /* Clear the configuration value. */
    midiHandle->configuration = 0U;
    midiHandle->alternate     = 0xffU;

    *handle = (class_handle_t)midiHandle;
    return error;
}

/*!
 * @brief De-initialize the device midi class.
 *
 * The function de-initializes the device midi class.
 *
 * @param handle The midi class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMidiDeinit(class_handle_t handle)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t error;

    midiHandle = (usb_device_midi_struct_t *)handle;

    if (NULL == midiHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* De-initialzie the endpoints. */
    error = USB_DeviceMidiEndpointsDeinit(midiHandle);
    /* Free the midi class handle. */
#if (defined(USB_DEVICE_CONFIG_RETURN_VALUE_CHECK) && (USB_DEVICE_CONFIG_RETURN_VALUE_CHECK > 0U))
    if (kStatus_USB_Success != USB_DeviceMidiFreeHandle(midiHandle))
    {
        return kStatus_USB_Error;
    }
#else
    (void)USB_DeviceMidiFreeHandle(midiHandle);
#endif
    return error;
}

/*!
 * @brief Send data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 * The function calls USB_DeviceSendRequest internally.
 *
 * @param handle The midi class handle got from usb_device_class_config_struct_t::classHandle.
 * @param ep     Endpoint index.
 * @param buffer The memory address to hold the data need to be sent.
 * @param length The data length need to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by
 * USB_DeviceMidiBulkIn.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceMidiSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t error = kStatus_USB_Error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    midiHandle = (usb_device_midi_struct_t *)handle;

    if (0U != midiHandle->inPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    midiHandle->inPipeBusy = 1U;

    if (0U != midiHandle->inPipeStall)
    {
        midiHandle->inPipeDataBuffer = buffer;
        midiHandle->inPipeDataLen    = length;
        return kStatus_USB_Success;
    }
    error = USB_DeviceSendRequest(midiHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        midiHandle->inPipeBusy = 0U;
    }
    return error;
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 * The function calls USB_DeviceRecvRequest internally.
 *
 * @param handle The midi class handle got from usb_device_class_config_struct_t::classHandle.
 * @param ep     Endpoint index.
 * @param buffer The memory address to save the received data.
 * @param length The data length want to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by
 * USB_DeviceMidiBulkOut.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceMidiRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_midi_struct_t *midiHandle;
    usb_status_t error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    midiHandle = (usb_device_midi_struct_t *)handle;

    if (0U != midiHandle->outPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    midiHandle->outPipeBusy = 1U;

    if (0U != midiHandle->outPipeStall)
    {
        midiHandle->outPipeDataBuffer = buffer;
        midiHandle->outPipeDataLen    = length;
        return kStatus_USB_Success;
    }
    error = USB_DeviceRecvRequest(midiHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        midiHandle->outPipeBusy = 0U;
    }
    return error;
}

#endif	//#if ((defined(USB_DEVICE_CONFIG_MIDI)) && (USB_DEVICE_CONFIG_MIDI > 0U))
