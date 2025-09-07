#include "audio_usb.h"
#include "audio_bus.h"
#include <math.h>

// Audio controls
static bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];
static int16_t volume_db[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];
static uint32_t sampFreq;
static uint8_t clkValid;
static audio_control_range_4_n_t(2) sampleFreqRng;

/* Public Functions */
void audio_usb_init(void) {
  sampFreq = SAMPLE_RATE;
  clkValid = 1;

  sampleFreqRng.wNumSubRanges = 1; // Corrected from 2 to 1 for a single sample rate
  sampleFreqRng.subrange[0].bMin = SAMPLE_RATE;
  sampleFreqRng.subrange[0].bMax = SAMPLE_RATE;
  sampleFreqRng.subrange[0].bRes = 0;
}

void audio_task(void) {
  static uint32_t start_ms = 0;
  uint32_t curr_ms = board_millis();
  
  // Send audio every 1ms
  if (curr_ms - start_ms >= 1) {
      start_ms = curr_ms;
      
    while(!rb_is_empty(&g_proc_to_usb_buffer)){
      uint8_t *buffer = (uint8_t *) rb_get_read_buffer(&g_proc_to_usb_buffer);
      tud_audio_write(buffer, AUDIO_PACKET_SIZE);
      rb_increase_read_index(&g_proc_to_usb_buffer);
    }
    // gpio_put(14, 0);
  }
}

/* TinyUSB Audio Class Callbacks */
// Copied directly from ecample code with minor changes

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) ep;

  return false;// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) itf;

  return false;// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  (void) itf;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // If request is for our feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
      case AUDIO_FU_CTRL_MUTE:
        // Request uses format layout 1
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

        mute[channelNum] = ((audio_control_cur_1_t *) pBuff)->bCur;
        if(mute[0] || mute[1]) {
          audio_mute = true;
        } else {
          audio_mute = false;
        }

        TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
        return true;

      case AUDIO_FU_CTRL_VOLUME:
        // Request uses format layout 2
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

        volume_db[channelNum] = (int16_t) ((audio_control_cur_2_t *) pBuff)->bCur;
        audio_volume_multiplier = powf(10.0f, (float)volume_db[channelNum] / 20.0f);

        TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume_db[channelNum], channelNum);
        return true;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }
  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) ep;

  //	return tud_control_xfer(rhport, p_request, &tmp, 1);

  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum;
  (void) ctrlSel;
  (void) itf;

  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // Input terminal (Microphone input)
  if (entityID == 1) {
    switch (ctrlSel) {
      case AUDIO_TE_CTRL_CONNECTOR: {
        // The terminal connector control only has a get request with only the CUR attribute.
        audio_desc_channel_cluster_t ret;

        // Those are dummy values for now
        ret.bNrChannels = 1;
        ret.bmChannelConfig = (audio_channel_config_t) 0;
        ret.iChannelNames = 0;

        TU_LOG2("    Get terminal connector\r\n");

        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));
      } break;

        // Unknown/Unsupported control selector
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
      case AUDIO_FU_CTRL_MUTE:
        // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
        // There does not exist a range parameter block for mute
        TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[channelNum], 1);

      case AUDIO_FU_CTRL_VOLUME:
        switch (p_request->bRequest) {
          case AUDIO_CS_REQ_CUR:
            TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume_db[channelNum], sizeof(volume_db[channelNum]));

          case AUDIO_CS_REQ_RANGE:
            TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

            audio_control_range_2_n_t(1)
                ret;

            ret.wNumSubRanges = 1;
            ret.subrange[0].bMin = -90; // dB
            ret.subrange[0].bMax = 0;   // dB, as the current hardware has no amplification capabilities
            ret.subrange[0].bRes = 1;   // 1 dB steps

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
        break;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Clock Source unit
  if (entityID == 4) {
    switch (ctrlSel) {
      case AUDIO_CS_CTRL_SAM_FREQ:
        // channelNum is always zero in this case
        switch (p_request->bRequest) {
          case AUDIO_CS_REQ_CUR:
            TU_LOG2("    Get Sample Freq.\r\n");
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

          case AUDIO_CS_REQ_RANGE:
            TU_LOG2("    Get Sample Freq. range\r\n");
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
        break;

      case AUDIO_CS_CTRL_CLK_VALID:
        // Only cur attribute exists for this request
        TU_LOG2("    Get Sample Freq. valid\r\n");
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

      // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  TU_LOG2("  Unsupported entity: %d\r\n", entityID);
  return false;// Yet not implemented
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;

  return true;
}