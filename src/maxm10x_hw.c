/*
 * maxm10x_hw.c
 *
 *  Created on: 15 mar. 2026
 *      Author: Ludo
 */

#include "maxm10x_hw.h"

#ifndef MAXM10X_DRIVER_DISABLE_FLAGS_FILE
#include "maxm10x_driver_flags.h"
#endif
#include "maxm10x.h"
#include "types.h"

#ifndef MAXM10X_DRIVER_DISABLE

/*** MAXM10X HW functions ***/

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_init(MAXM10X_HW_configuration_t* configuration) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    UNUSED(configuration);
    return status;
}

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_de_init(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    return status;
}

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_send_message(uint8_t* message, uint32_t message_size_bytes) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    UNUSED(message);
    UNUSED(message_size_bytes);
    return status;
}

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_start_rx(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    return status;
}

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_stop_rx(void) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    return status;
}

/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_set_backup_voltage(uint8_t state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    UNUSED(state);
    return status;
}
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
MAXM10X_status_t __attribute__((weak)) MAXM10X_HW_get_backup_voltage(uint8_t* state) {
    // Local variables.
    MAXM10X_status_t status = MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED;
    /* To be implemented */
    UNUSED(state);
    return status;
}
#endif

#endif /* MAXM10X_DRIVER_DISABLE */
