/*
 * maxm10x.c
 *
 *  Created on: 15 mar. 2026
 *      Author: Ludo
 */

#include "maxm10x.h"

#ifndef MAXM10X_DRIVER_DISABLE_FLAGS_FILE
#include "maxm10x_driver_flags.h"
#endif
#include "error.h"
#include "maths.h"
#include "maxm10x_hw.h"
#include "strings.h"
#include "types.h"

#ifndef MAXM10X_DRIVER_DISABLE

/*** MAXM10X local macros ***/

#define MAXM10X_UART_BAUD_RATE                          9600

#define MAXM10X_UBX_MSG_OVERHEAD_SIZE_BYTES             8

#define MAXM10X_UBX_CHECKSUM_OVERHEAD_SIZE_BYTES        4
#define MAXM10X_UBX_CHECKSUM_OFFSET_BYTES               2

#define MAXM10X_UBX_CONFIGURATION_VALUE_SIZE_BYTES_MAX  8

#define MAXM10X_UBX_CFG_VALSET_PAYLOAD_SIZE_BYTES_MAX   16
#define MAXM10X_UBX_CFG_TP5_PAYLOAD_SIZE_BYTES          32

#define MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES               128
#define MAXM10X_NMEA_RX_BUFFER_DEPTH                    2

#define MAXM10X_NMEA_CHAR_MESSAGE_START                 '$'
#define MAXM10X_NMEA_CHAR_CHECKSUM_START                '*'
#define MAXM10X_NMEA_CHAR_SEPARATOR                     ','
#define MAXM10X_NMEA_CHAR_END                           STRING_CHAR_LF

#define MAXM10X_NMEA_GGA_NORTH                          'N'
#define MAXM10X_NMEA_GGA_SOUTH                          'S'
#define MAXM10X_NMEA_GGA_EAST                           'E'
#define MAXM10X_NMEA_GGA_WEST                           'W'
#define MAXM10X_NMEA_GGA_METERS                         'M'

#define MAXM10X_TIMEPULSE_FREQUENCY_HZ_MAX              10000000

#if (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 1)
#define MAXM10X_ALTITUDE_STABILITY_THRESHOLD            MAXM10X_DRIVER_ALTITUDE_STABILITY_THRESHOLD
#endif
#if (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2)
#define MAXM10X_ALTITUDE_STABILITY_THRESHOLD            (maxm10x_ctx.acquisition.altitude_stability_threshold)
#endif

/*** MAXM10X local structures ***/

/*******************************************************************/
typedef enum {
    MAXM10X_NMEA_MESSAGE_INDEX_DTM = 0,
    MAXM10X_NMEA_MESSAGE_INDEX_GBS,
    MAXM10X_NMEA_MESSAGE_INDEX_GGA,
    MAXM10X_NMEA_MESSAGE_INDEX_GLL,
    MAXM10X_NMEA_MESSAGE_INDEX_GNS,
    MAXM10X_NMEA_MESSAGE_INDEX_GRS,
    MAXM10X_NMEA_MESSAGE_INDEX_GSA,
    MAXM10X_NMEA_MESSAGE_INDEX_GST,
    MAXM10X_NMEA_MESSAGE_INDEX_GSV,
    MAXM10X_NMEA_MESSAGE_INDEX_RLM,
    MAXM10X_NMEA_MESSAGE_INDEX_RMC,
    MAXM10X_NMEA_MESSAGE_INDEX_VLW,
    MAXM10X_NMEA_MESSAGE_INDEX_VTG,
    MAXM10X_NMEA_MESSAGE_INDEX_ZDA,
    MAXM10X_NMEA_MESSAGE_INDEX_LAST
} MAXM10X_nmea_message_index_t;

/*******************************************************************/
typedef enum {
    MAXM10X_NMEA_ZDA_FIELD_INDEX_MESSAGE = 0,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_TIME,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_DAY,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_MONTH,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_YEAR,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_LTZH,
    MAXM10X_NMEA_ZDA_FIELD_INDEX_LTZN,
} MAXM10X_nmea_zda_field_index_t;

/*******************************************************************/
typedef enum {
    MAXM10X_NMEA_ZDA_FIELD_SIZE_MESSAGE = 5,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_TIME = 9,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_DAY = 2,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_MONTH = 2,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_YEAR = 4,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_LTZH = 2,
    MAXM10X_NMEA_ZDA_FIELD_SIZE_LTZN = 2
} NMEA_zda_field_size_t;

/*******************************************************************/
typedef enum {
    MAXM10X_NMEA_GGA_FIELD_INDEX_MESSAGE = 0,
    MAXM10X_NMEA_GGA_FIELD_INDEX_TIME,
    MAXM10X_NMEA_GGA_FIELD_INDEX_LAT,
    MAXM10X_NMEA_GGA_FIELD_INDEX_NS,
    MAXM10X_NMEA_GGA_FIELD_INDEX_LONG,
    MAXM10X_NMEA_GGA_FIELD_INDEX_EW,
    MAXM10X_NMEA_GGA_FIELD_INDEX_QUALITY,
    MAXM10X_NMEA_GGA_FIELD_INDEX_NUMSV,
    MAXM10X_NMEA_GGA_FIELD_INDEX_HDOP,
    MAXM10X_NMEA_GGA_FIELD_INDEX_ALT,
    MAXM10X_NMEA_GGA_FIELD_INDEX_U_ALT,
    MAXM10X_NMEA_GGA_FIELD_INDEX_SEP,
    MAXM10X_NMEA_GGA_FIELD_INDEX_U_SEP,
    MAXM10X_NMEA_GGA_FIELD_INDEX_DIFF_AGE,
    MAXM10X_NMEA_GGA_FIELD_INDEX_DIFF_STATION
} NMEA_gga_field_index_t;

/*******************************************************************/
typedef enum {
    MAXM10X_NMEA_GGA_FIELD_SIZE_MESSAGE = 5,
    MAXM10X_NMEA_GGA_FIELD_SIZE_TIME = 9,
    MAXM10X_NMEA_GGA_FIELD_SIZE_LAT = 10,
    MAXM10X_NMEA_GGA_FIELD_SIZE_NS = 1,
    MAXM10X_NMEA_GGA_FIELD_SIZE_LONG = 11,
    MAXM10X_NMEA_GGA_FIELD_SIZE_EW = 1,
    MAXM10X_NMEA_GGA_FIELD_SIZE_QUALITY = 1,
    MAXM10X_NMEA_GGA_FIELD_SIZE_NUM_SV = 0,
    MAXM10X_NMEA_GGA_FIELD_SIZE_HDOP = 0,
    MAXM10X_NMEA_GGA_FIELD_SIZE_ALT = 0,
    MAXM10X_NMEA_GGA_FIELD_SIZE_U_ALT = 1,
    MAXM10X_NMEA_GGA_FIELD_SIZE_SEP = 0,
    MAXM10X_NMEA_GGA_FIELD_SIZE_U_SEP = 1,
    MAXM10X_NMEA_GGA_FIELD_SIZE_DIFF_AGE = 0,
    MAXM10X_NMEA_GGA_FIELD_SIZE_DIFF_STATION = 0
} NMEA_gga_field_size_t;

/*******************************************************************/
typedef struct {
    // Buffers.
    volatile char_t nmea_buffer[MAXM10X_NMEA_RX_BUFFER_DEPTH][MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES];
    volatile uint8_t nmea_char_idx;
    volatile uint8_t nmea_buffer_idx_write;
    volatile uint8_t nmea_buffer_idx_ready;
    volatile uint8_t nmea_frame_received_flag;
    // Local data.
    MAXM10X_acquisition_t acquisition;
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    MAXM10X_time_t gps_time;
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    MAXM10X_position_t gps_position;
#endif
#if ((defined MAXM10X_DRIVER_GPS_DATA_POSITION) && (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0))
    // Altitude stability filter.
    uint8_t same_altitude_count;
    uint32_t previous_altitude;
#endif
} MAXM10X_context_t;

/*** MAXM10X local global variables ***/

static MAXM10X_context_t maxm10x_ctx;

/*** MAXM10X local functions ***/

/*******************************************************************/
#define _MAXM10X_check_field_size(field_size) { if ((char_idx - separator_idx) != (field_size + 1)) goto errors; }

/*******************************************************************/
#define _MAXM10X_check_string_status(void) { if (string_status != STRING_SUCCESS) goto errors; }

/*******************************************************************/
static void _MAXM10X_rx_irq_callback(uint8_t message_byte) {
    // Store new byte.
    maxm10x_ctx.nmea_buffer[maxm10x_ctx.nmea_buffer_idx_write][maxm10x_ctx.nmea_char_idx] = (char_t) message_byte;
    // Manage character index.
    maxm10x_ctx.nmea_char_idx++;
    // Check buffer size and NMEA ending marker.
    if ((message_byte == MAXM10X_NMEA_CHAR_END) || (maxm10x_ctx.nmea_char_idx >= MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        // Check ending marker.
        if (message_byte == MAXM10X_NMEA_CHAR_END) {
            // Update flag.
            maxm10x_ctx.nmea_frame_received_flag = 1;
            maxm10x_ctx.nmea_buffer_idx_ready = maxm10x_ctx.nmea_buffer_idx_write;
        }
        // Switch buffer.
        maxm10x_ctx.nmea_buffer_idx_write = (uint8_t) ((maxm10x_ctx.nmea_buffer_idx_write + 1) % MAXM10X_NMEA_RX_BUFFER_DEPTH);
        maxm10x_ctx.nmea_char_idx = 0;
        // Ask for processing.
        if ((message_byte == MAXM10X_NMEA_CHAR_END) && (maxm10x_ctx.acquisition.process_callback != NULL)) {
            maxm10x_ctx.acquisition.process_callback();
        }
    }
}

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static void _MAXM10X_reset_time(MAXM10X_time_t* gps_time) {
    // Reset all fields to 0.
    (gps_time->year) = 0;
    (gps_time->month) = 0;
    (gps_time->date) = 0;
    (gps_time->hours) = 0;
    (gps_time->minutes) = 0;
    (gps_time->seconds) = 0;
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static void _MAXM10X_reset_position(MAXM10X_position_t* gps_position) {
    // Reset all fields to 0.
    (gps_position->lat_degrees) = 0;
    (gps_position->lat_minutes) = 0;
    (gps_position->lat_seconds) = 0;
    (gps_position->lat_north_flag) = 0;
    (gps_position->long_degrees) = 0;
    (gps_position->long_minutes) = 0;
    (gps_position->long_seconds) = 0;
    (gps_position->long_east_flag) = 0;
    (gps_position->altitude) = 0;
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static void _MAXM10X_copy_time(MAXM10X_time_t* source, MAXM10X_time_t* destination) {
    // Copy data.
    (destination->year) = (source->year);
    (destination->month) = (source->month);
    (destination->date) = (source->date);
    (destination->hours) = (source->hours);
    (destination->minutes) = (source->minutes);
    (destination->seconds) = (source->seconds);
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static void _MAXM10X_copy_position(MAXM10X_position_t* source, MAXM10X_position_t* destination) {
    // Copy data.
    (destination->lat_degrees) = (source->lat_degrees);
    (destination->lat_minutes) = (source->lat_minutes);
    (destination->lat_seconds) = (source->lat_seconds);
    (destination->lat_north_flag) = (source->lat_north_flag);
    (destination->long_degrees) = (source->long_degrees);
    (destination->long_minutes) = (source->long_minutes);
    (destination->long_seconds) = (source->long_seconds);
    (destination->long_east_flag) = (source->long_east_flag);
    (destination->altitude) = (source->altitude);
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static uint8_t _MAXM10X_check_time(MAXM10X_time_t* gps_time) {
    // Local variables.
    uint8_t time_valid_flag = 0;
    // Check time fields.
    if (((gps_time->date) > 0) && ((gps_time->date) < 32) &&
        ((gps_time->month) > 0) && ((gps_time->month) < 13) &&
        ((gps_time->year) > 2023) && ((gps_time->year) < 2094) &&
        ((gps_time->hours) < 24) &&
        ((gps_time->minutes) < 60) &&
        ((gps_time->seconds) < 60))
    {
        time_valid_flag = 1;
    }
    return time_valid_flag;
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static uint8_t _MAXM10X_check_position(MAXM10X_position_t* gps_position) {
    // Local variables.
    uint8_t position_valid_flag = 0;
    // Check position fields.
    if ((gps_position->lat_degrees < 90) && (gps_position->lat_minutes < 60) &&(gps_position->lat_seconds < 100000) &&
        (gps_position->long_degrees < 180) && (gps_position->long_minutes < 60) && (gps_position->long_seconds < 100000))
    {
        position_valid_flag = 1;
    }
    return position_valid_flag;
}
#endif

/*******************************************************************/
static void _MAXM10X_compute_ubx_checksum(uint8_t* ubx_command, uint8_t payload_length) {
    // Local variables.
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    uint32_t idx = 0;
    // See algorithm on p.44 of MAX-M10 programming manual.
    for (idx = MAXM10X_UBX_CHECKSUM_OFFSET_BYTES; idx < ((uint32_t) (MAXM10X_UBX_CHECKSUM_OFFSET_BYTES + MAXM10X_UBX_CHECKSUM_OVERHEAD_SIZE_BYTES + payload_length)); idx++) {
        ck_a = ck_a + ubx_command[idx];
        ck_b = ck_b + ck_a;
    }
    // Fill two last bytes of the UBX message with CK_A and CK_B.
    ubx_command[idx + 0] = ck_a;
    ubx_command[idx + 1] = ck_b;
}

/*******************************************************************/
static void _MAXM10X_compute_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck, uint8_t* compute_success_flag) {
    // Local variables.
    uint8_t message_start_char_idx = 0;
    uint8_t checksum_start_char_idx = 0;
    uint8_t checksum_idx = 0;
    // Reset value and flag.
    (*ck) = 0;
    (*compute_success_flag) = 0;
    // Get message start index.
    while ((nmea_rx_buf[message_start_char_idx] != MAXM10X_NMEA_CHAR_MESSAGE_START) && (message_start_char_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        message_start_char_idx++;
    }
    // Get checksum start index.
    checksum_start_char_idx = message_start_char_idx;
    while ((nmea_rx_buf[checksum_start_char_idx] != MAXM10X_NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        checksum_start_char_idx++;
    }
    if (checksum_start_char_idx >= MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES) goto errors;
    // Compute checksum.
    for (checksum_idx = (message_start_char_idx + 1); checksum_idx < checksum_start_char_idx; checksum_idx++) {
        // Exclusive OR of all characters between '$' and '*'.
        (*ck) ^= (uint8_t) nmea_rx_buf[checksum_idx];
    }
    // Update output flag.
    (*compute_success_flag) = 1;
errors:
    return;
}

/*******************************************************************/
static void _MAXM10X_get_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck, uint8_t* get_success_flag) {
    // Local variables.
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_start_char_idx = 0;
    int32_t ck_value = 0;
    // Reset value and flag.
    (*ck) = 0;
    (*get_success_flag) = 0;
    // Get checksum start index.
    while ((nmea_rx_buf[checksum_start_char_idx] != MAXM10X_NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        checksum_start_char_idx++;
    }
    if (checksum_start_char_idx >= MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES) goto errors;
    // Convert hexadecimal to value.
    string_status = STRING_string_to_integer(&(nmea_rx_buf[checksum_start_char_idx + 1]), STRING_FORMAT_HEXADECIMAL, 2, &ck_value);
    if (string_status != STRING_SUCCESS) goto errors;
    // Cast to byte.
    (*ck) = (uint8_t) ck_value;
    // Update output flag.
    (*get_success_flag) = 1;
errors:
    return;
}

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static void _MAXM10X_parse_nmea_zda(char_t* nmea_rx_buf, MAXM10X_time_t* gps_time, uint8_t* decode_success_flag) {
    // Local variables.
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_get_success_flag = 0;
    uint8_t checksum_compute_success_flag = 0;
    uint8_t received_checksum = 0;
    uint8_t computed_checksum = 0;
    uint8_t last_field_parsed = 0;
    MAXM10X_nmea_zda_field_index_t field_idx = 0;
    uint8_t char_idx = 0;
    uint8_t separator_idx = 0;
    int32_t value = 0;
    // Reset flag.
    (*decode_success_flag) = 0;
    // Compute checksums.
    _MAXM10X_get_nmea_checksum(nmea_rx_buf, &received_checksum, &checksum_get_success_flag);
    _MAXM10X_compute_nmea_checksum(nmea_rx_buf, &computed_checksum, &checksum_compute_success_flag);
    // Verify checksum.
    if ((checksum_get_success_flag == 0) || (checksum_compute_success_flag == 0) || (computed_checksum != received_checksum)) goto errors;
    // Search NMEA start character.
    while ((nmea_rx_buf[separator_idx] != MAXM10X_NMEA_CHAR_MESSAGE_START) && (separator_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        separator_idx++;
        char_idx++;
    }
    // Extract NMEA data (see ZDA message format on p.34 of MAX-M10 programming manual).
    while ((nmea_rx_buf[char_idx] != MAXM10X_NMEA_CHAR_END) && (char_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        // Check if separator is found.
        if (nmea_rx_buf[char_idx] == MAXM10X_NMEA_CHAR_SEPARATOR) {
            // Get current field.
            switch (field_idx) {
            // Field 0 = address = <ID><message>.
            case MAXM10X_NMEA_ZDA_FIELD_INDEX_MESSAGE:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_ZDA_FIELD_SIZE_MESSAGE);
                // Check if message = 'ZDA'.
                if ((nmea_rx_buf[separator_idx + 3] != 'Z') || (nmea_rx_buf[separator_idx + 4] != 'D') || (nmea_rx_buf[separator_idx + 5] != 'A')) goto errors;
                break;
            // Field 1 = time = hhmmss.ss.
            case MAXM10X_NMEA_ZDA_FIELD_INDEX_TIME:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_ZDA_FIELD_SIZE_TIME);
                // Parse hours.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_time->hours = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_time->minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 5]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_time->seconds = (uint8_t) value;
                break;
            // Field 2 = day = dd.
            case MAXM10X_NMEA_ZDA_FIELD_INDEX_DAY:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_ZDA_FIELD_SIZE_DAY);
                // Parse day.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_time->date = (uint8_t) value;
                break;
            // Field 3 = month = mm.
            case MAXM10X_NMEA_ZDA_FIELD_INDEX_MONTH:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_ZDA_FIELD_SIZE_MONTH);
                // Parse month.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_time->month = (uint8_t) value;
                break;
            // Field 4 = year = yyyy.
            case MAXM10X_NMEA_ZDA_FIELD_INDEX_YEAR:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_ZDA_FIELD_SIZE_YEAR);
                // Parse year.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 4, &value);
                _MAXM10X_check_string_status();
                gps_time->year = (uint16_t) value;
                // Update local flag.
                last_field_parsed = 1;
                break;
            // Unused or unknown fields.
            default:
                break;
            }
            // Increment field index and update separator.
            field_idx++;
            separator_idx = char_idx;
        }
        // Increment character index.
        char_idx++;
    }
    if (last_field_parsed != 0) {
        // Check if time is valid.
        (*decode_success_flag) = _MAXM10X_check_time(gps_time);
    }
errors:
    return;
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static void _MAXM10X_parse_nmea_gga(char_t* nmea_rx_buf, MAXM10X_position_t* gps_position, uint8_t* decode_success_flag) {
    // Local variables
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_get_success_flag = 0;
    uint8_t checksum_compute_success_flag = 0;
    uint8_t received_checksum = 0;
    uint8_t computed_checksum = 0;
    uint8_t last_field_parsed = 0;
    NMEA_gga_field_index_t field_idx = 0;
    uint8_t char_idx = 0;
    uint8_t separator_idx = 0;
    uint8_t alt_field_size = 0;
    uint8_t alt_number_of_digits = 0;
    int32_t value = 0;
    // Reset flag.
    (*decode_success_flag) = 0;
    // Compute checksums.
    _MAXM10X_get_nmea_checksum(nmea_rx_buf, &received_checksum, &checksum_get_success_flag);
    _MAXM10X_compute_nmea_checksum(nmea_rx_buf, &computed_checksum, &checksum_compute_success_flag);
    // Verify checksum.
    if ((checksum_get_success_flag == 0) || (checksum_compute_success_flag == 0) || (computed_checksum != received_checksum)) goto errors;
    // Search NMEA start character.
    while ((nmea_rx_buf[separator_idx] != MAXM10X_NMEA_CHAR_MESSAGE_START) && (separator_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        separator_idx++;
        char_idx++;
    }
    // Extract NMEA data (see GGA message format on p.24 of MAX-M10 programming manual).
    while ((nmea_rx_buf[char_idx] != MAXM10X_NMEA_CHAR_END) && (char_idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES)) {
        // Check if separator is found.
        if (nmea_rx_buf[char_idx] == MAXM10X_NMEA_CHAR_SEPARATOR) {
            // Get current field.
            switch (field_idx) {
            // Field 0 = address = <ID><message>.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_MESSAGE:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_MESSAGE);
                // Check if message = 'GGA'.
                if ((nmea_rx_buf[separator_idx + 3] != 'G') || (nmea_rx_buf[separator_idx + 4] != 'G') || (nmea_rx_buf[separator_idx + 5] != 'A')) goto errors;
                break;
            // Field 2 = latitude = ddmm.mmmmm.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_LAT:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_LAT);
                // Parse degrees.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_position->lat_degrees = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_position->lat_minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 6]), STRING_FORMAT_DECIMAL, 5, &value);
                _MAXM10X_check_string_status();
                gps_position->lat_seconds = (uint32_t) value;
                break;
            // Field 3 = N or S.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_NS:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_NS);
                // Parse north flag.
                switch (nmea_rx_buf[separator_idx + 1]) {
                case MAXM10X_NMEA_GGA_NORTH:
                    (*gps_position).lat_north_flag = 1;
                    break;
                case MAXM10X_NMEA_GGA_SOUTH:
                    (*gps_position).lat_north_flag = 0;
                    break;
                default:
                    goto errors;
                }
                break;
            // Field 4 = longitude = dddmm.mmmmm.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_LONG:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_LONG);
                // Parse degrees.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 3, &value);
                _MAXM10X_check_string_status();
                gps_position->long_degrees = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 4]), STRING_FORMAT_DECIMAL, 2, &value);
                _MAXM10X_check_string_status();
                gps_position->long_minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 7]), STRING_FORMAT_DECIMAL, 5, &value);
                _MAXM10X_check_string_status();
                gps_position->long_seconds = (uint32_t) value;
                break;
            // Field 5 = E or W.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_EW:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_EW);
                // Parse east flag.
                switch (nmea_rx_buf[separator_idx + 1]) {
                case MAXM10X_NMEA_GGA_EAST:
                    (*gps_position).long_east_flag = 1;
                    break;
                case MAXM10X_NMEA_GGA_WEST:
                    (*gps_position).long_east_flag = 0;
                    break;
                default:
                    goto errors;
                }
                break;
            // Field 9 = altitude.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_ALT:
                // Get field length.
                alt_field_size = (uint8_t) ((char_idx - separator_idx) - 1);
                // Check field length.
                if (alt_field_size == 0) goto errors;
                // Get number of digits of integer part (search dot).
                for (alt_number_of_digits = 0; alt_number_of_digits < alt_field_size; alt_number_of_digits++) {
                    if (nmea_rx_buf[separator_idx + 1 + alt_number_of_digits] == STRING_CHAR_DOT) {
                        break; // Dot found, stop counting integer part length.
                    }
                }
                // Compute integer part.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, alt_number_of_digits, &value);
                _MAXM10X_check_string_status();
                gps_position->altitude = (uint32_t) value;
                // Rounding operation if fractional part exists.
                if ((char_idx - (separator_idx + alt_number_of_digits) - 1) >= 2) {
                    // Convert tenth part.
                    string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + alt_number_of_digits + 2]), STRING_FORMAT_DECIMAL, 1, &value);
                    _MAXM10X_check_string_status();
                    if (value >= 5) {
                        (*gps_position).altitude++;
                    }
                }
                break;
            // Field 10 = altitude unit.
            case MAXM10X_NMEA_GGA_FIELD_INDEX_U_ALT:
                // Check field length.
                _MAXM10X_check_field_size(MAXM10X_NMEA_GGA_FIELD_SIZE_U_ALT);
                // Parse altitude unit.
                if (nmea_rx_buf[separator_idx + 1] != MAXM10X_NMEA_GGA_METERS) goto errors;
                // Update local flag.
                last_field_parsed = 1;
                break;
                // Unused or unknown fields.
            default:
                break;
            }
            // Increment field index and update separator.
            field_idx++;
            separator_idx = char_idx;
        }
        // Increment character index.
        char_idx++;
    }
    if (last_field_parsed != 0) {
        // Check if time is valid.
        (*decode_success_flag) = _MAXM10X_check_position(gps_position);
    }
errors:
    return;
}
#endif

/*******************************************************************/
static MAXM10X_status_t _MAXM10X_write_configuration(uint32_t key, uint8_t* value, uint8_t value_size_bytes) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // See p.55 for UBX message format.
    uint8_t ubx_cfg_valset[MAXM10X_UBX_MSG_OVERHEAD_SIZE_BYTES + MAXM10X_UBX_CFG_VALSET_PAYLOAD_SIZE_BYTES_MAX] = {
        0xB5, 0x62,                                                                                 // Preamble.
        0x06, 0x8A,                                                                                 // UBX message class and ID.
        0x00, 0x00,                                                                                 // Length.
        0x00,                                                                                       // Version.
        0x01,                                                                                       // Layers.
        0x00, 0x00,                                                                                 // Reserved.
        (uint8_t) (key >> 0), (uint8_t) (key >> 8), (uint8_t) (key >> 16), (uint8_t) (key >> 24),   // Key.
        0, 0, 0, 0, 0, 0, 0, 0,                                                                     // Value (1, 2, 4 or 8 bytes).
        0x00, 0x00                                                                                  // UBX checksum.
    };
    uint8_t payload_size = (8 + value_size_bytes);
    uint8_t idx = 0;
    // Update value.
    for (idx = 0; idx < value_size_bytes; idx++) {
        ubx_cfg_valset[14 + idx] = value[value_size_bytes - 1 - idx];
    }
    // Compute checksum.
    _MAXM10X_compute_ubx_checksum(ubx_cfg_valset, payload_size);
    // Send message.
    status = MAXM10X_HW_send_message(ubx_cfg_valset, (MAXM10X_UBX_MSG_OVERHEAD_SIZE_BYTES + payload_size));
    if (status != MAXM10X_SUCCESS) goto errors;
    // Delay between messages.
    status = MAXM10X_HW_delay_milliseconds(100);
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static MAXM10X_status_t _MAXM10X_select_nmea_messages(uint32_t nmea_message_id_mask) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    uint8_t nmea_message_uart_rate_value = 0;
    uint8_t nmea_idx = 0;
    uint8_t idx = 0;
    // See p.133 for NMEA messages key.
    uint32_t nmea_message_uart_rate_key[MAXM10X_NMEA_MESSAGE_INDEX_LAST] = {
        0x209100A7, // DTM.
        0x209100DE, // GBS.
        0x209100BB, // GGA.
        0x209100CA, // GLL.
        0x209100B6, // GNS.
        0x209100CF, // GRS.
        0x209100C0, // GSA.
        0x209100D4, // GST.
        0x209100C5, // GSV.
        0x20910401, // RLM.
        0x209100AC, // RMC.
        0x209100E8, // VLW.
        0x209100B1, // VTG.
        0x209100D9  // ZDA.
    };
    // Send commands.
    for (nmea_idx = 0; nmea_idx < MAXM10X_NMEA_MESSAGE_INDEX_LAST; nmea_idx++) {
        // Enable or disable message.
        nmea_message_uart_rate_value = ((nmea_message_id_mask & (0b1 << nmea_idx)) != 0) ? 1 : 0;
        // Send message.
        status = _MAXM10X_write_configuration(nmea_message_uart_rate_key[idx], &nmea_message_uart_rate_value, 1);
        if (status != MAXM10X_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*** MAXM10X functions ***/

/*******************************************************************/
MAXM10X_status_t MAXM10X_init(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    MAXM10X_HW_configuration_t hw_config;
    uint8_t buffer_idx = 0;
    uint32_t idx = 0;
    // Init context.
    for (buffer_idx = 0; buffer_idx < MAXM10X_NMEA_RX_BUFFER_DEPTH; buffer_idx++) {
        for (idx = 0; idx < MAXM10X_NMEA_RX_BUFFER_SIZE_BYTES; idx++)
            maxm10x_ctx.nmea_buffer[buffer_idx][idx] = 0;
    }
    maxm10x_ctx.nmea_buffer_idx_write = 0;
    maxm10x_ctx.nmea_buffer_idx_ready = 0;
    maxm10x_ctx.nmea_frame_received_flag = 0;
    maxm10x_ctx.acquisition.gps_data = MAXM10X_GPS_DATA_LAST;
    maxm10x_ctx.acquisition.process_callback = NULL;
    maxm10x_ctx.acquisition.completion_callback = NULL;
    // Init hardware interface.
    hw_config.uart_baud_rate = MAXM10X_UART_BAUD_RATE;
    hw_config.rx_irq_callback = &_MAXM10X_rx_irq_callback;
    status = MAXM10X_HW_init(&hw_config);
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_de_init(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Release hardware interface.
    status = MAXM10X_HW_de_init();
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_start_acquisition(MAXM10X_acquisition_t* acquisition) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Check state.
    if (maxm10x_ctx.acquisition.gps_data != MAXM10X_GPS_DATA_LAST) {
        status = MAXM10X_ERROR_ACQUISITION_RUNNING;
        goto errors;
    }
    // Reset context.
    maxm10x_ctx.nmea_frame_received_flag = 0;
#if ((defined MAXM10X_DRIVER_GPS_DATA_POSITION) && (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0))
    maxm10x_ctx.same_altitude_count = 0;
    maxm10x_ctx.previous_altitude = 0;
#endif
    // Check parameters.
    if (acquisition == NULL) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (((acquisition->process_callback) == NULL) || ((acquisition->completion_callback) == NULL)) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy acquisition parameters locally.
    maxm10x_ctx.acquisition.gps_data = (acquisition->gps_data);
    maxm10x_ctx.acquisition.completion_callback = (acquisition->completion_callback);
    maxm10x_ctx.acquisition.process_callback = (acquisition->process_callback);
#if ((defined MAXM10X_DRIVER_GPS_DATA_POSITION) && (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2))
    maxm10x_ctx.acquisition.altitude_stability_threshold = (acquisition->altitude_stability_threshold);
#endif
    // Select NMEA messages.
    switch (maxm10x_ctx.acquisition.gps_data) {
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    case MAXM10X_GPS_DATA_TIME:
        // Reset structure.
        _MAXM10X_reset_time(&(maxm10x_ctx.gps_time));
        // Select ZDA message to get complete date and time.
        status = _MAXM10X_select_nmea_messages(0b1 << MAXM10X_NMEA_MESSAGE_INDEX_ZDA);
        if (status != MAXM10X_SUCCESS) goto errors;
        break;
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    case MAXM10X_GPS_DATA_POSITION:
        // Reset structure.
        _MAXM10X_reset_position(&(maxm10x_ctx.gps_position));
        // Select GGA message to get complete position.
        status = _MAXM10X_select_nmea_messages(0b1 << MAXM10X_NMEA_MESSAGE_INDEX_GGA);
        if (status != MAXM10X_SUCCESS) goto errors;
        break;
#endif
    default:
        status = MAXM10X_ERROR_ACQUISITION_DATA;
        goto errors;
    }
    // Start NMEA frames reception.
    status = MAXM10X_HW_start_rx();
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_stop_acquisition(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Release driver.
    maxm10x_ctx.acquisition.gps_data = MAXM10X_GPS_DATA_LAST;
    // Stop NMEA frames reception.
    status = MAXM10X_HW_stop_rx();
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
MAXM10X_status_t MAXM10X_process(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    MAXM10X_acquisition_status_t acquisition_status = MAXM10X_ACQUISITION_STATUS_FAIL;
    uint8_t decode_success_flag = 0;
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    MAXM10X_time_t gps_time;
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    MAXM10X_position_t gps_position;
#endif
    // Check flag.
    if (maxm10x_ctx.nmea_frame_received_flag == 0) goto errors;
    // Clear flag.
    maxm10x_ctx.nmea_frame_received_flag = 0;
    // Reset structures.
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    _MAXM10X_reset_time(&gps_time);
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    _MAXM10X_reset_position(&gps_position);
#endif
    // Decode incoming NMEA message.
    switch (maxm10x_ctx.acquisition.gps_data) {
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    case MAXM10X_GPS_DATA_TIME:
        // Parse buffer.
        _MAXM10X_parse_nmea_zda((char_t*) maxm10x_ctx.nmea_buffer[maxm10x_ctx.nmea_buffer_idx_ready], &gps_time, &decode_success_flag);
        // Check decoding result.
        if (decode_success_flag != 0) {
            // Copy data and update status.
            _MAXM10X_copy_time(&gps_time, &(maxm10x_ctx.gps_time));
            acquisition_status = MAXM10X_ACQUISITION_STATUS_FOUND;
        }
        break;
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    case MAXM10X_GPS_DATA_POSITION:
        // Parse buffer.
        _MAXM10X_parse_nmea_gga((char_t*) maxm10x_ctx.nmea_buffer[maxm10x_ctx.nmea_buffer_idx_ready], &gps_position, &decode_success_flag);
        // Check decoding result.
        if (decode_success_flag != 0) {
            // Copy data and update status.
            _MAXM10X_copy_position(&gps_position, &(maxm10x_ctx.gps_position));
            acquisition_status = MAXM10X_ACQUISITION_STATUS_FOUND;
#if (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0)
            // Directly exit if the filter is disabled.
            if (MAXM10X_ALTITUDE_STABILITY_THRESHOLD >= 2) {
                // Manage altitude stability count.
                if ((maxm10x_ctx.gps_position.altitude) == maxm10x_ctx.previous_altitude) {
                    maxm10x_ctx.same_altitude_count++;
                    // Compare to threshold.
                    if (maxm10x_ctx.same_altitude_count >= (MAXM10X_ALTITUDE_STABILITY_THRESHOLD - 1)) {
                        // Update status.
                        acquisition_status = MAXM10X_ACQUISITION_STATUS_STABLE;
                    }
                }
                else {
                    maxm10x_ctx.same_altitude_count = 0;
                }
                // Update previous altitude.
                maxm10x_ctx.previous_altitude = (maxm10x_ctx.gps_position.altitude);
            }
#endif
        }
        break;
#endif
    default:
        status = MAXM10X_ERROR_ACQUISITION_DATA;
        goto errors;
    }
    // Call callback in case of success.
    if (acquisition_status != MAXM10X_ACQUISITION_STATUS_FAIL) {
        maxm10x_ctx.acquisition.completion_callback(acquisition_status);
    }
errors:
    return status;
}

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
MAXM10X_status_t MAXM10X_get_time(MAXM10X_time_t* gps_time) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Check parameter.
    if (gps_time == NULL) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy data.
    _MAXM10X_copy_time(&(maxm10x_ctx.gps_time), gps_time);
errors:
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
MAXM10X_status_t MAXM10X_get_position(MAXM10X_position_t* gps_position) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Check parameters.
    if (gps_position == NULL) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy data.
    _MAXM10X_copy_position(&(maxm10x_ctx.gps_position), gps_position);
errors:
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t MAXM10X_set_backup_voltage(uint8_t state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Call hardware function.
    status = MAXM10X_HW_set_backup_voltage(state);
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t MAXM10X_get_backup_voltage(uint8_t* state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    // Check parameter.
    if (state == NULL) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Call hardware function.
    status = MAXM10X_HW_get_backup_voltage(state);
errors:
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_TIMEPULSE
/*******************************************************************/
MAXM10X_status_t MAXM10X_set_timepulse(MAXM10X_timepulse_configuration_t* timepulse_config) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_SUCCESS;
    uint8_t value [MAXM10X_UBX_CONFIGURATION_VALUE_SIZE_BYTES_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    // Check parameters.
    if (timepulse_config == NULL) {
        status = MAXM10X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((timepulse_config->frequency_hz) > MAXM10X_TIMEPULSE_FREQUENCY_HZ_MAX) {
        status = MAXM10X_ERROR_TIMEPULSE_FREQUENCY;
        goto errors;
    }
    if ((timepulse_config->duty_cycle_percent) > 100) {
        status = MAXM10X_ERROR_TIMEPULSE_DUTY_CYCLE;
        goto errors;
    }
    // Use frequency in Hz.
    value[0] = 1;
    status = _MAXM10X_write_configuration(0x20050023, value, 1);
    if (status != MAXM10X_SUCCESS) goto errors;
    // Use duty cycle in percent.
    value[0] = 0;
    status = _MAXM10X_write_configuration(0x20050030, value, 1);
    if (status != MAXM10X_SUCCESS) goto errors;
    // Use same parameters when locked.
    status = _MAXM10X_write_configuration(0x10050009, value, 1);
    if (status != MAXM10X_SUCCESS) goto errors;
    // Frequency.
    value[0] = (uint8_t) ((timepulse_config->frequency_hz) >> 0);
    value[1] = (uint8_t) ((timepulse_config->frequency_hz) >> 8);
    value[2] = (uint8_t) ((timepulse_config->frequency_hz) >> 16);
    value[3] = (uint8_t) ((timepulse_config->frequency_hz) >> 24);
    status = _MAXM10X_write_configuration(0x40050024, value, 4);
    if (status != MAXM10X_SUCCESS) goto errors;
    // Duty cycle.
    value[0] = (timepulse_config->duty_cycle_percent);
    value[1] = 0;
    value[2] = 0;
    value[3] = 0;
    status = _MAXM10X_write_configuration(0x5005002A, value, 8);
    if (status != MAXM10X_SUCCESS) goto errors;
    // State.
    value[0] = ((timepulse_config->active) == 0) ? 0 : 1;
    status = _MAXM10X_write_configuration(0x10050007, value, 1);
    if (status != MAXM10X_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#endif /* MAXM10X_DRIVER_DISABLE */
