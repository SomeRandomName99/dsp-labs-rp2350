#ifndef AUDIO_USB_H
#define AUDIO_USB_H

#include <tusb.h>
#include <bsp/board_api.h>

/* Constants and Macros */
#define SAMPLE_RATE            CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE
#define AUDIO_BYTES_PER_SAMPLE 2 // 16-bit
#define AUDIO_CHANNELS         1
#define AUDIO_SAMPLES_PER_MS   (SAMPLE_RATE / 1000)
#define AUDIO_PACKET_SAMPLES   (AUDIO_SAMPLES_PER_MS * AUDIO_BYTES_PER_SAMPLE * AUDIO_CHANNELS)
#define AUDIO_PACKET_SIZE      AUDIO_PACKET_SAMPLES * AUDIO_BYTES_PER_SAMPLE

/* Shared buffer */
volatile extern uint8_t usb_buffer[AUDIO_PACKET_SIZE];

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