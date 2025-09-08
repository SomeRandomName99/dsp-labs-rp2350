#ifndef AUDIO_USB_H
#define AUDIO_USB_H

#include <tusb.h>
#include <bsp/board_api.h>

#include "audio_bus.h"

/* Public Function Prototypes */
void audio_usb_init(void);
void audio_task(void);

/* TinyUSB Audio Class Callback Prototypes */
// They need to be public for TinyUSB to be able to call them
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff);
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff);
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff);
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request);
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request);
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request);
bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request);

#endif //AUDIO_USB_H