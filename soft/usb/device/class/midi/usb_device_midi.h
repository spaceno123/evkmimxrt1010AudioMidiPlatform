/*
 * usb_device_midi.h
 *
 *  Created on: 2024/08/29
 *      Author: M.Akino
 */

#ifndef __USB_DEVICE_MIDI_H__
#define __USB_DEVICE_MIDI_H__

/*!
 * @addtogroup usb_device_midi_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief MIDI(Audio) device class code */
#define USB_DEVICE_CONFIG_MIDI_CLASS_CODE (0x01U)

/*! @brief MIDI(Audio) device subclass code */
#define USB_DEVICE_MIDI_STREAM_SUBCLASS  (0x03U)
#define USB_DEVICE_MIDI_CONTROL_SUBCLASS (0x01U)

/*! @brief MIDI(Audio) device class-specific descriptor type */
#define USB_DESCRIPTOR_TYPE_MIDI_CS_INTERFACE (0x24U)
#define USB_DESCRIPTOR_TYPE_MIDI_CS_ENDPOINT  (0x25U)

/*! @brief MIDI(Audio) device MS class-specific interface descriptor subtype */
#define USB_DESCRIPTOR_SUBTYPE_MS_DESCRIPTOR_UNDEFINED (0x00U)
#define USB_DESCRIPTOR_SUBTYPE_MS_HEADER               (0x01U)
#define USB_DESCRIPTOR_SUBTYPE_MS_MIDI_IN_JACK         (0x02U)
#define USB_DESCRIPTOR_SUBTYPE_MS_MIDI_OUT_JACK        (0x03U)
#define USB_DESCRIPTOR_SUBTYPE_MS_ELEMENT              (0x04U)

/*! @brief MIDI(Audio) device MS class-specific endpoint descriptor subtype */
#define USB_DESCRIPTOR_SUBTYPE_MS_UNDEFINED (0x00U)
#define USB_DESCRIPTOR_SUBTYPE_MS_GENERAL   (0x01U)

/*! @brief MIDI(Audio) device MS MIDI IN and OUT Jack types */
#define USB_DESCRIPTOR_TYPE_MS_JACK_TYPE_UNDEFINED (0x00U)
#define USB_DESCRIPTOR_TYPE_MS_EMBEDDED            (0x01U)
#define USB_DESCRIPTOR_TYPE_MS_EXTERNAL            (0x02U)

/*! @brief Available common EVENT types in MIDI(Audio) class callback */
typedef enum _usb_device_midi_event
{
    kUSB_DeviceMidiEventSendResponse = 0x01U, /*!< Send data completed or cancelled etc*/
    kUSB_DeviceMidiEventRecvResponse,         /*!< Data received or cancelled etc*/
} usb_device_midi_event_t;

/*! @brief The audio device class status structure */
typedef struct _usb_device_midi_struct
{
    usb_device_handle handle;                       /*!< The device handle */
    usb_device_class_config_struct_t *configStruct; /*!< The configuration of the class. */
    usb_device_interface_struct_t *interfaceHandle; /*!< Current stream interface handle */
    uint8_t *inPipeDataBuffer;                      /*!< IN pipe data buffer backup when stall */
    uint32_t inPipeDataLen;                         /*!< IN pipe data length backup when stall  */
    uint8_t *outPipeDataBuffer;                     /*!< OUT pipe data buffer backup when stall */
    uint32_t outPipeDataLen;                        /*!< OUT pipe data length backup when stall  */
    uint8_t configuration;                          /*!< Current configuration */
    uint8_t interfaceNumber;                        /*!< The interface number of the class */
    uint8_t alternate;                              /*!< Current alternate of the interface */
    uint8_t inPipeBusy;                             /*!< IN pipe busy flag */
    uint8_t outPipeBusy;                            /*!< OUT pipe busy flag */
    uint8_t inPipeStall;                            /*!< IN pipe stall flag */
    uint8_t outPipeStall;                           /*!< OUT pipe stall flag */
} usb_device_midi_struct_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the MIDI class.
 *
 * This function is used to initialize the MIDI class. This function only can be called by #USB_DeviceClassInit.
 *
 * @param[in] controllerId   The controller ID of the USB IP. See the enumeration #usb_controller_index_t.
 * @param[in] config          The class configuration information.
 * @param[out] handle          An parameter used to return pointer of the MIDI class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceMidiInit(uint8_t controllerId,
                                       usb_device_class_config_struct_t *config,
                                       class_handle_t *handle);

/*!
 * @brief Deinitializes the device MIDI class.
 *
 * The function deinitializes the device MIDI class. This function only can be called by #USB_DeviceClassDeinit.
 *
 * @param[in] handle The MIDI class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceMidiDeinit(class_handle_t handle);

/*!
 * @brief Handles the event passed to the MIDI class.
 *
 * This function handles the event passed to the MIDI class. This function only can be called by #USB_DeviceClassEvent.
 *
 * @param[in] handle          The MIDI class handle received from the usb_device_class_config_struct_t::classHandle.
 * @param[in] event           The event codes. See the enumeration usb_device_class_event_t.
 * @param[in,out] param           The parameter type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe is stalled by the caller.
 */
extern usb_status_t USB_DeviceMidiEvent(void *handle, uint32_t event, void *param);

/*!
 * @name USB device MIDI class APIs
 * @{
 */

/*!
 * @brief Sends data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 * The function calls #USB_DeviceSendRequest internally.
 *
 * @param[in] handle The MIDI class handle received from usb_device_class_config_struct_t::classHandle.
 * @param[in] ep     Endpoint index.
 * @param[in] buffer The memory address to hold the data need to be sent.
 * @param[in] length The data length to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The function can only be called in the same context.
 *
 * @note The return value indicates whether the sending request is successful or not. The transfer done is notified by
 * usb_device_midi_bulk_in.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is received through the
 * endpoint
 * callback).
 */
extern usb_status_t USB_DeviceMidiSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 * The function calls #USB_DeviceRecvRequest internally.
 *
 * @param[in] handle The MIDI class handle received from the usb_device_class_config_struct_t::classHandle.
 * @param[in] ep     Endpoint index.
 * @param[in] buffer The memory address to save the received data.
 * @param[in] length The data length to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The function can only be called in the same context.
 *
 * @note The return value indicates whether the receiving request is successful or not. The transfer done is notified by
 * usb_device_midi_bulk_out.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is received through the
 * endpoint
 * callback).
 */
extern usb_status_t USB_DeviceMidiRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __USB_DEVICE_MIDI_H__ */
