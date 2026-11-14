# Local copy of espressif/usb_host_uvc 2.5.1

Copied from the ESP Component Registry (Apache-2.0) with one fix, marked
"espVT/JC8012 demo change" in the sources.

**Bulk payload parser.** In a bulk stream every payload transfer (at most
`dwMaxPayloadTransferSize` bytes) starts with its own UVC payload header. The original
parser only removed the header at the start and at the end of a frame. Cameras that
split a frame into several payloads, for example the UGREEN HDMI-to-USB grabber
(MacroSilicon, 16 KB payloads), got the intermediate 12-byte headers written into the
JPEG data, and the hardware JPEG decoder rejected every frame with
"marker parsed by the decoder is not supported by the hardware".

The new parser in `uvc_bulk.c` tracks the bytes left in the current payload and strips
every header; frames end on the EOF bit, a short packet after an EOF payload, or a
Frame-ID toggle. The payload size is stored at stream open and at commit
(`uvc_host.c`, `uvc_control.c`, `private_include/uvc_types_priv.h`).

The same code is still present in usb_host_uvc 2.6.0.
