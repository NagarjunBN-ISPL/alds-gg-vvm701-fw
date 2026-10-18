#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * rsp_tunnel.h — the cloud's raw command tunnel to a dispenser (rsp, kfactor).
 *
 * The CLOUD builds every dispenser command — an RSP (price) write, a k-factor
 * write, a status read, anything — and the gateway only carries it:
 *
 *   <base>/rsp/imei/<hw_id>/in       BlueSky and Yenen
 *   <base>/kfactor/imei/<hw_id>/in   BlueSky only (Yenen has no k-factor)
 *       the frame as hex text (rsp_tunnel_decode), put on the bus ONCE in
 *       that dispenser's line mode
 *   .../out
 *       the pump's reply exactly as received, lowercase comma hex — or "404"
 *
 * Nothing in between looks at the bytes: no frame, address or checksum test,
 * no idle gate, no retry, no read-back, no rebuilt frame. That is the April
 * 2026 pilot's contract (executeRemoteCommand / executeYenenRSP), restored on
 * 2026-10-18 by user decision — the same tunnel cng-cpil and hsd-snf carry.
 * Making a write safe (not mid-sale, for one) is the cloud's job: it can send
 * the status read through this same tunnel first and decide on the reply.
 *
 * "404" means only that there is no reply to forward: the payload is not a hex
 * frame, the queue is full, the bus stayed busy, the pump said nothing, or the
 * line never went quiet. Never a cut frame.
 */

/* Longest command accepted, in bytes. The pilot's buffer was 64; the longest
 * real command is the 14-byte Yenen RSP write. Longer is refused whole — a
 * frame cut short is a different frame. Fits the uint8_t length below. */
#define RSP_TUNNEL_REQ_MAX        64

/* Longest reply forwarded, in bytes. The longest real one is the 51-byte Yenen
 * Command 2 reply. More than this is a line talking past any frame: "404". */
#define RSP_TUNNEL_RESP_MAX       128

/* Reply text buffer: two digits per byte, a comma between bytes, and the NUL —
 * exactly 3 characters per byte. */
#define RSP_TUNNEL_REPLY_TEXT_MAX (RSP_TUNNEL_RESP_MAX * 3)

/* Commands waiting for the bus. A whole-station price change pushed at once
 * (6 BlueSky dispensers = 12 nozzles) fits with room to spare; a command that
 * finds the queue full is answered "404" straight away. */
#define RSP_TUNNEL_QUEUE_MAX      16

typedef enum {
    RSP_TUNNEL_RSP = 0,       /* <base>/rsp/imei/<hw_id>/in      */
    RSP_TUNNEL_KFACTOR,       /* <base>/kfactor/imei/<hw_id>/in  */
} RspTunnelKind_t;

typedef struct {
    uint8_t unit;                        /* dispenser slot, 0-based          */
    uint8_t kind;                        /* RspTunnelKind_t                  */
    uint8_t len;                         /* bytes in frame, 1..REQ_MAX       */
    uint8_t frame[RSP_TUNNEL_REQ_MAX];
} RspTunnelCmd_t;

/* The MQTT payload -> the bytes to send. Takes every form the pilot's
 * strtol() parser read correctly, and the forms a publisher produces on its
 * own (the same rules as cng-cpil's du_hex_decode_frame):
 *
 *   f5,02,a4,68,80,b2,09            the pilot's comma form, any case
 *   f5,2,a4,68,80,b2,9              unpadded tokens, read as strtol read them
 *   0xF5,0x02,0xA4,...              0x prefixes
 *   F5 02 A4 / F5:02:A4 / F5-02-A4 / F502A4
 *   "f5,02,a4,..."                  any of those as ONE JSON string
 *   {"Status":"lock","NZ1":"02,30,38,..."}
 *                                   the pilot's Yenen wrapper: the first
 *                                   "NZ<n>" string value is the frame, every
 *                                   other key ("Status") is ignored, as the
 *                                   pilot ignored it
 *
 * Returns the frame length, or 0 — nothing is sent — for anything else: a
 * character that is neither hex nor a separator, an odd run of digits
 * ("021"), a bare "0x", an object without an "NZ<n>" string, or more than
 * out_max bytes. Where the pilot read a bad token as 00, this refuses: a byte
 * the cloud did not write must never reach a pump. Pure, no I/O. */
size_t rsp_tunnel_decode(const char *payload, uint8_t *out, size_t out_max);

/* One exchange on the RS485 bus. Call with the bus lock held and the
 * dispenser's line mode already set (rs485_bus.h).
 *
 * Sends `req` once, then reads the way the pilot did: up to
 * RS485_RECV_TIMEOUT_MS for the first byte, then on until the line has been
 * quiet for RS485_RECV_TIMEOUT_MS — whatever the pump sends, however it is laid
 * out. A cap of 3x that window ends a line that never goes quiet.
 *
 * `reply` (RSP_TUNNEL_REPLY_TEXT_MAX is always enough) gets the bytes as
 * lowercase comma hex and true is returned; or "404" and false when nothing
 * came back, more than RSP_TUNNEL_RESP_MAX bytes came, or the line never went
 * quiet. Feeds the task watchdog while it waits. */
bool rsp_tunnel_exchange(const uint8_t *req, size_t len,
                         char *reply, size_t reply_size);

/* The waiting commands, oldest first. handle_msg() pushes and
 * execute_pending() pops, both on the main task, so there is no locking.
 * push stores nothing and returns false when the queue is full or len is
 * outside 1..RSP_TUNNEL_REQ_MAX. */
bool rsp_tunnel_push(uint8_t unit, uint8_t kind, const uint8_t *frame, size_t len);
bool rsp_tunnel_pop(RspTunnelCmd_t *out);
