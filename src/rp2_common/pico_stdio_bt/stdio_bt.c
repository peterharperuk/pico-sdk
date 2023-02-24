#include <stdio.h>
#include <inttypes.h>

#include "btstack.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_bt.h"
#include "pico/mutex.h"

#define RFCOMM_SERVER_CHANNEL 1

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

#define BT_BUF_SIZE 128

typedef struct bt_buf {
    uint8_t data[BT_BUF_SIZE];
    uint16_t read_pos;
    uint16_t write_pos;
    mutex_t lock;
} bt_buffer_t;

// todo: locking
static bt_buffer_t bt_buffer_in;
static bt_buffer_t bt_buffer_out;

#if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
static void (*chars_available_callback)(void*);
static void *chars_available_param;
#endif

#define SAFE_PRINTF(...) if (rfcomm_channel_id == 0) printf(__VA_ARGS__)

// Write one entry to the buffer
static bool write_buf(bt_buffer_t *buf, char data) {
    bool result = false;
    if (mutex_try_enter(&buf->lock, NULL)) {
        uint16_t new_write_pos = buf->write_pos + 1;
        if (new_write_pos >= sizeof(buf->data)) new_write_pos = 0;
        if (new_write_pos != buf->read_pos) {
            buf->data[buf->write_pos] = data;
            buf->write_pos = new_write_pos;
            result = true;
        }
        mutex_exit(&buf->lock);
    }
    return result;
}

// Read one entry from the buffer
static bool read_buf(bt_buffer_t *buf, uint8_t *data) {
    bool result = false;
    if (mutex_try_enter(&buf->lock, NULL)) {
        uint16_t new_read_pos = buf->read_pos + 1;
        if (buf->read_pos != buf->write_pos) {
            if (new_read_pos >= sizeof(buf->data)) new_read_pos = 0;
            *data = buf->data[buf->read_pos];
            buf->read_pos = new_read_pos;
            result = true;
        }
        mutex_exit(&buf->lock);
    }
    return result;
}

// Get a pointer to continuous data that can be read from the buffer
// Leaves with the mutex owned unless zero is returned
static uint16_t read_ptr(bt_buffer_t *buf, uint8_t **data) {
    uint16_t data_available = 0;
    if (mutex_try_enter(&buf->lock, NULL)) {
        if (buf->read_pos < buf->write_pos) {
            data_available = buf->write_pos - buf->read_pos;
            *data = &buf->data[buf->read_pos];
            buf->read_pos = buf->write_pos;
        } else if (buf->read_pos > buf->write_pos) {
            data_available = sizeof(buf->data) - buf->read_pos;
            *data = &buf->data[buf->read_pos];
            buf->read_pos = 0;
        } else {
            mutex_exit(&buf->lock);
        }
    }
    return data_available;
}

// Write to the input buffer
static uint16_t write_in(uint8_t *buffer, uint16_t length) {
    uint16_t amount_written = 0;
    while(amount_written < length) {
        if (!write_buf(&bt_buffer_in, buffer[amount_written])) break;
        amount_written++;
    #if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
        if (chars_available_callback) chars_available_callback(chars_available_param);
    #endif
    }
    return amount_written;
}

// Read from the input buffer
static uint16_t read_in(uint8_t *buffer, uint16_t max_length) {
    uint16_t amount_read = 0;
    while(amount_read < max_length) {
        if (!read_buf(&bt_buffer_in, &buffer[amount_read])) break;
        amount_read++;
    }
    return amount_read;
}

// Write to the output buffer
static uint16_t write_out(const char *buffer, uint16_t length) {
    uint16_t amount_written = 0;
    if (rfcomm_channel_id) {
        while(amount_written < length) {
            if (!write_buf(&bt_buffer_out, buffer[amount_written])) break;
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
            amount_written++;
        }
    }
    return amount_written;

}

// get pointer to the output buffer
static uint16_t get_out_lock(uint8_t **buffer) {
    return read_ptr(&bt_buffer_out, buffer);
}

// Called when finished with the buffer
static void get_out_unlock(void) {
    mutex_exit(&bt_buffer_out.lock);
}

static void stdio_bt_out_chars(const char *buf, int length) {
    write_out(buf, length);
}

static int stdio_bt_in_chars(char *buf, int length) {
    int count = read_in(buf, length);
    return (count > 0) ? count : PICO_ERROR_NO_DATA;
}

#if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
void stdio_usb_set_chars_available_callback(void (*fn)(void*), void *param) {
    chars_available_callback = fn;
    chars_available_param = param;
}
#endif

stdio_driver_t stdio_bt = {
    .out_chars = stdio_bt_out_chars,
    .in_chars = stdio_bt_in_chars,
#if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
    .set_chars_available_callback = stdio_bt_set_chars_available_callback,
#endif
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_BT_DEFAULT_CRLF
#endif
};

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    SAFE_PRINTF("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    SAFE_PRINTF("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    SAFE_PRINTF("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    SAFE_PRINTF("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        SAFE_PRINTF("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        stdio_set_driver_enabled(&stdio_bt, true);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW: {
                    uint16_t len;
                    do {
                        uint8_t *data;
                        len = get_out_lock(&data);
                        if (len > 0) {
                            rfcomm_send(rfcomm_channel_id, data, len);
                            get_out_unlock();
                        }
                    } while(len > 0);
                    break;
                }
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    stdio_set_driver_enabled(&stdio_bt, false);
                    rfcomm_channel_id = 0;
                    SAFE_PRINTF("RFCOMM channel closed\n");
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            write_in(packet, size);
            break;

        default:
            break;
    }
}

#if !CYW43_ENABLE_BLUETOOTH
#error bluetooth needs to be enabled
#endif

bool stdio_bt_init(void) {
    mutex_init(&bt_buffer_in.lock);
    mutex_init(&bt_buffer_out.lock);

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "PICO SDIO");
    sdp_register_service(spp_service_buffer);

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("Pico stdio 00:00:00:00:00:00");

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);
}