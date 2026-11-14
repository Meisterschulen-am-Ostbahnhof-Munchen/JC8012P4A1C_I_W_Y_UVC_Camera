/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <string.h> // For memcpy

#include "esp_log.h"

#include "uvc_stream.h" // For uvc_host_stream_pause()
#include "uvc_types_priv.h"
#include "uvc_check_priv.h"
#include "uvc_frame_priv.h"
#include "uvc_critical_priv.h"

static const char *TAG = "uvc-bulk";

static void bulk_complete_frame(uvc_stream_t *uvc_stream)
{
    // Clear the current frame before invoking the callback so no later packet can append to it.
    UVC_ENTER_CRITICAL();
    uvc_host_frame_t *this_frame = uvc_stream->dynamic.current_frame;
    uvc_stream->dynamic.current_frame = NULL;

    const bool invoke_fb_callback = (uvc_stream->dynamic.streaming && uvc_stream->constant.frame_cb && this_frame &&
                                     !uvc_stream->single_thread.skip_current_frame);
    UVC_EXIT_CRITICAL();

    bool return_frame = true;
    if (invoke_fb_callback) {
        return_frame = uvc_stream->constant.frame_cb(this_frame, uvc_stream->constant.cb_arg);
    }
    if (this_frame && return_frame) {
        uvc_host_frame_return(uvc_stream, this_frame);
    }
}

static void bulk_add_frame_data(uvc_stream_t *uvc_stream, const uint8_t *data, size_t data_len)
{
    if (uvc_stream->single_thread.skip_current_frame || data_len == 0) {
        return;
    }

    uvc_host_frame_t *current_frame = UVC_ATOMIC_LOAD(uvc_stream->dynamic.current_frame);
    if (!current_frame) {
        ESP_LOGE(TAG, "missing current frame while appending bulk payload");
        uvc_stream->single_thread.skip_current_frame = true;
        return;
    }

    esp_err_t ret = uvc_frame_add_data(current_frame, data, data_len);
    if (ret != ESP_OK) {
        // Frame buffer overflow
        uvc_stream->single_thread.skip_current_frame = true;

        // Inform the user about the overflow
        uvc_host_stream_callback_t stream_cb = uvc_stream->constant.stream_cb;
        if (stream_cb) {
            const uvc_host_stream_event_data_t event = {
                .type = UVC_HOST_FRAME_BUFFER_OVERFLOW,
            };
            stream_cb(&event, uvc_stream->constant.cb_arg);
        }
    }
}

/**
 * @brief Callback function for handling Bulk USB transfers from a UVC camera.
 *
 * This function processes Bulk transfer packets, managing frame data according to a state machine. The characteristics
 * and requirements of Bulk transfers are as follows:
 *
 * - **CRC Included**: Ensures no errors in frame data.
 * - **ACK Mechanism**: Missed packets are retransmitted, ensuring reliable data delivery.
 * - **Packet Headers**: Not all packets contain headers; the end of a transfer is indicated by a short packet
 *   (less than the maximum packet size), and the next packet usually contains a header.
 *
 * To process these packets, a state machine is implemented to track the next expected Bulk packet type:
 * - Start of Frame (SoF), usually followed by data packets in the same transfer
 * - Data packets (no header)
 * - End of Frame (EoF)
 *
 * The function handles USB transfer statuses, manages frame buffers, and invokes user-defined callbacks for
 * completed frames.
 *
 * @param[in] transfer Pointer to the completed USB transfer structure.
 */
void bulk_transfer_callback(usb_transfer_t *transfer)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    uvc_stream_t *uvc_stream = (uvc_stream_t *)transfer->context;

    // Check USB transfer status
    switch (transfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        break;
    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
    case USB_TRANSFER_STATUS_ERROR:
    case USB_TRANSFER_STATUS_OVERFLOW:
    case USB_TRANSFER_STATUS_STALL:
        // On Bulk errors we stop the stream
        //@todo Stall, error and overflow errors should be propagated to the user
        ESP_ERROR_CHECK(uvc_host_stream_pause(uvc_stream)); // This should never fail
        break;
    case USB_TRANSFER_STATUS_TIMED_OUT:
    case USB_TRANSFER_STATUS_SKIPPED: // Should never happen to BULK transfer
    default:
        assert(false);
    }

    if (!UVC_ATOMIC_LOAD(uvc_stream->dynamic.streaming)) {
        return; // If the streaming was turned off, we don't have to do anything
    }

    /*
     * espVT/JC8012 demo change: payload-aware bulk parser.
     *
     * In a bulk stream every payload transfer (at most dwMaxPayloadTransferSize bytes)
     * starts with its own UVC payload header. A payload ends after
     * dwMaxPayloadTransferSize bytes or with a short packet. The original parser only
     * stripped the header at the start and at the end of a frame, so cameras that split
     * a frame into several payloads (e.g. MacroSilicon HDMI grabbers, 16 KB payloads)
     * got the intermediate headers written into the image data.
     */
    const uint8_t *data = transfer->data_buffer;
    const size_t len = transfer->actual_num_bytes;
    const bool short_transfer = transfer->actual_num_bytes < transfer->data_buffer_size;
    const uint32_t payload_size = uvc_stream->single_thread.bulk_payload_size;
    size_t pos = 0;

    while (pos < len) {
        if (uvc_stream->single_thread.bulk_payload_left == 0) {
            // A new payload starts here: parse its header
            const uvc_payload_header_t *hdr = (const uvc_payload_header_t *)(data + pos);
            if (!uvc_frame_payload_header_validate(hdr, len - pos)) {
                ESP_LOGD(TAG, "invalid UVC payload header, %02x %02x", data[pos], data[pos + 1]);
                uvc_stream->single_thread.skip_current_frame = true;
                break; // drop the rest of this transfer
            }
            const uint8_t fid = hdr->bmHeaderInfo.frame_id;
            if (uvc_stream->single_thread.bulk_frame_open && fid != uvc_stream->single_thread.current_frame_id) {
                // Frame ID toggled without EOF: the previous frame is complete
                bulk_complete_frame(uvc_stream);
                uvc_stream->single_thread.bulk_frame_open = false;
            }
            if (!uvc_stream->single_thread.bulk_frame_open) {
                // Start a new frame
                uvc_stream->single_thread.current_frame_id = fid;
                uvc_stream->single_thread.skip_current_frame = false;
                uvc_stream->single_thread.bulk_frame_open = true;
                UVC_ENTER_CRITICAL();
                const bool need_new_frame = (uvc_stream->dynamic.streaming && !uvc_stream->dynamic.current_frame);
                UVC_EXIT_CRITICAL();
                if (need_new_frame) {
                    uvc_stream->dynamic.current_frame = uvc_frame_get_empty(uvc_stream);
                    if (uvc_stream->dynamic.current_frame == NULL) {
                        uvc_stream->single_thread.skip_current_frame = true;
                        uvc_host_stream_callback_t stream_cb = uvc_stream->constant.stream_cb;
                        if (stream_cb) {
                            const uvc_host_stream_event_data_t event = {.type = UVC_HOST_FRAME_BUFFER_UNDERFLOW};
                            stream_cb(&event, uvc_stream->constant.cb_arg);
                        }
                    }
                } else if (uvc_stream->dynamic.current_frame) {
                    uvc_frame_reset(uvc_stream->dynamic.current_frame);
                }
            }
            if (hdr->bmHeaderInfo.error) {
                uvc_stream->single_thread.skip_current_frame = true;
            }
            uvc_stream->single_thread.bulk_eof_pending = hdr->bmHeaderInfo.end_of_frame;
            pos += hdr->bHeaderLength;
            uvc_stream->single_thread.bulk_payload_left =
                (payload_size > hdr->bHeaderLength) ? payload_size - hdr->bHeaderLength : UINT32_MAX;
        }

        size_t chunk = len - pos;
        if (chunk > uvc_stream->single_thread.bulk_payload_left) {
            chunk = uvc_stream->single_thread.bulk_payload_left;
        }
        bulk_add_frame_data(uvc_stream, data + pos, chunk);
        pos += chunk;
        uvc_stream->single_thread.bulk_payload_left -= chunk;

        if (uvc_stream->single_thread.bulk_payload_left == 0 && uvc_stream->single_thread.bulk_eof_pending) {
            bulk_complete_frame(uvc_stream);
            uvc_stream->single_thread.bulk_frame_open = false;
            uvc_stream->single_thread.bulk_eof_pending = false;
        }
    }

    if (short_transfer) {
        // A short (or zero-length) packet ends the current payload
        if (uvc_stream->single_thread.bulk_eof_pending && uvc_stream->single_thread.bulk_frame_open) {
            bulk_complete_frame(uvc_stream);
            uvc_stream->single_thread.bulk_frame_open = false;
        }
        uvc_stream->single_thread.bulk_eof_pending = false;
        uvc_stream->single_thread.bulk_payload_left = 0;
    }

    if (UVC_ATOMIC_LOAD(uvc_stream->dynamic.streaming)) {
        usb_host_transfer_submit(transfer); // Restart the transfer
    }
}
