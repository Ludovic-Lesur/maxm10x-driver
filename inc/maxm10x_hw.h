/*
 * maxm10x_hw.h
 *
 *  Created on: 15 mar. 2026
 *      Author: Ludo
 */

#ifndef MAXM10X_DRIVER_DISABLE_FLAGS_FILE
#include "maxm10x_driver_flags.h"
#endif
#include "maxm10x.h"
#include "types.h"

#ifndef __MAXM10X_HW_H__
#define __MAXM10X_HW_H__

#ifndef MAXM10X_DRIVER_DISABLE

/*** MAXM10X HW structures ***/

/*!******************************************************************
 * \fn MAXM10X_HW_rx_irq_cb_t
 * \brief Byte reception interrupt callback.
 *******************************************************************/
typedef void (*MAXM10X_HW_rx_irq_cb_t)(uint8_t message_byte);

/*!******************************************************************
 * \struct MAXM10X_HW_configuration_t
 * \brief MAXM10X hardware interface parameters.
 *******************************************************************/
typedef struct {
    uint32_t uart_baud_rate;
    MAXM10X_HW_rx_irq_cb_t rx_irq_callback;
} MAXM10X_HW_configuration_t;

/*** MAXM10X HW functions ***/

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_init(MAXM10X_HW_configuration_t* configuration)
 * \brief Init MAXM10X hardware interface.
 * \param[in]   configuration: Pointer to the hardware interface parameters structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_init(MAXM10X_HW_configuration_t* configuration);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_de_init(void)
 * \brief Release MAXM10X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_de_init(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_send_message(uint8_t* message, uint32_t message_size_bytes)
 * \brief Send a message over the MAXM10X control interface.
 * \param[in]   message: Bytes array to send.
 * \param[in]   message_size_bytes: Number of bytes to send.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_send_message(uint8_t* message, uint32_t message_size_bytes);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_start_rx(void)
 * \brief Start NMEA frames reception.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_start_rx(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_stop_rx(void)
 * \brief Stop NMEA frames reception.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_stop_rx(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]   delay_ms: Delay to wait in ms.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_delay_milliseconds(uint32_t delay_ms);

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_set_backup_voltage(uint8_t state)
 * \brief Set GPS backup voltage state.
 * \param[in]   state: 0 to turn off, turn on otherwise.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_set_backup_voltage(uint8_t state);
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_HW_get_backup_voltage(uint8_t* state)
 * \brief Get GPS backup voltage state.
 * \param[in]   none
 * \param[out]  state: Pointer to the current VBCKP pin state.
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_HW_get_backup_voltage(uint8_t* state);
#endif

#endif /* MAXM10X_DRIVER_DISABLE */

#endif /* __MAXM10X_HW_H__ */
