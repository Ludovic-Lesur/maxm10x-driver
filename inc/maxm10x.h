/*
 * maxm10x.h
 *
 *  Created on: 15 mar. 2026
 *      Author: Ludo
 */

#ifndef __MAXM10X_H__
#define __MAXM10X_H__

#ifndef MAXM10X_DRIVER_DISABLE_FLAGS_FILE
#include "maxm10x_driver_flags.h"
#endif
#include "error.h"
#include "strings.h"
#include "types.h"

/*** MAXM10X structures ***/

/*!******************************************************************
 * \enum MAXM10X_status_t
 * \brief MAXM10X driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    MAXM10X_SUCCESS = 0,
    MAXM10X_ERROR_NULL_PARAMETER,
    MAXM10X_ERROR_ACQUISITION_DATA,
    MAXM10X_ERROR_ACQUISITION_RUNNING,
    MAXM10X_ERROR_TIMEPULSE_FREQUENCY,
    MAXM10X_ERROR_TIMEPULSE_DUTY_CYCLE,
    // Low level drivers errors.
    MAXM10X_ERROR_HW_FUNCTION_NOT_IMPLEMENTED,
    MAXM10X_ERROR_BASE_GPIO = ERROR_BASE_STEP,
    MAXM10X_ERROR_BASE_UART = (MAXM10X_ERROR_BASE_GPIO + MAXM10X_DRIVER_GPIO_ERROR_BASE_LAST),
    MAXM10X_ERROR_BASE_DELAY = (MAXM10X_ERROR_BASE_UART + MAXM10X_DRIVER_UART_ERROR_BASE_LAST),
    MAXM10X_ERROR_BASE_STRING = (MAXM10X_ERROR_BASE_DELAY + MAXM10X_DRIVER_DELAY_ERROR_BASE_LAST),
    // Last base value.
    MAXM10X_ERROR_BASE_LAST = (MAXM10X_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} MAXM10X_status_t;

#ifndef MAXM10X_DRIVER_DISABLE

/*!******************************************************************
 * \enum MAXM10X_acquisition_status_t
 * \brief MAXM10X GPS acquisition result.
 *******************************************************************/
typedef enum {
    MAXM10X_ACQUISITION_STATUS_FAIL = 0,
    MAXM10X_ACQUISITION_STATUS_FOUND,
#if (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0)
    MAXM10X_ACQUISITION_STATUS_STABLE,
#endif
    MAXM10X_ACQUISITION_STATUS_LAST
} MAXM10X_acquisition_status_t;

/*!******************************************************************
 * \enum MAXM10X_data_t
 * \brief MAXM10X data list.
 *******************************************************************/
typedef enum {
#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
    MAXM10X_GPS_DATA_TIME,
#endif
#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
    MAXM10X_GPS_DATA_POSITION,
#endif
    MAXM10X_GPS_DATA_LAST
} MAXM10X_gps_data_t;

/*!******************************************************************
 * \fn MAXM10X_process_cb_t
 * \brief MAXM10X driver process callback.
 *******************************************************************/
typedef void (*MAXM10X_process_cb_t)(void);

/*!******************************************************************
 * \fn MAXM10X_completion_cb_t
 * \brief MAXM10X acquisition completion callback.
 *******************************************************************/
typedef void (*MAXM10X_completion_cb_t)(MAXM10X_acquisition_status_t acquisition_status);

/*!******************************************************************
 * \struct MAXM10X_acquisition_t
 * \brief MAXM10X acquisition parameters.
 *******************************************************************/
typedef struct {
    MAXM10X_gps_data_t gps_data;
    MAXM10X_process_cb_t process_callback;
    MAXM10X_completion_cb_t completion_callback;
#if ((defined MAXM10X_DRIVER_GPS_DATA_POSITION) && (MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2))
    uint8_t altitude_stability_threshold;
#endif
} MAXM10X_acquisition_t;

/*!******************************************************************
 * \struct MAXM10X_time_t
 * \brief GPS time structure.
 *******************************************************************/
typedef struct {
    // Date.
    uint16_t year;
    uint8_t month;
    uint8_t date;
    // Time.
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} MAXM10X_time_t;

/*!******************************************************************
 * \struct MAXM10X_position_t
 * \brief GPS position data. Note: seconds are expressed in (fractional part of minutes * 100000).
 *******************************************************************/
typedef struct {
    // Latitude.
    uint8_t lat_degrees;
    uint8_t lat_minutes;
    uint32_t lat_seconds;
    uint8_t lat_north_flag;
    // Longitude.
    uint8_t long_degrees;
    uint8_t long_minutes;
    uint32_t long_seconds;
    uint8_t long_east_flag;
    // Altitude.
    uint32_t altitude;
} MAXM10X_position_t;

/*!******************************************************************
 * \struct MAXM10X_timepulse_configuration_t
 * \brief Timepulse signal parameters.
 *******************************************************************/
typedef struct {
    uint8_t active;
    uint32_t frequency_hz;
    uint8_t duty_cycle_percent;
} MAXM10X_timepulse_configuration_t;

/*** MAXM10X functions ***/

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_init(void)
 * \brief Init MAXM10X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_init(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_de_init(void)
 * \brief Release MAXM10X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_de_init(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_start_acquisition(MAXM10X_acquisition_t* acquisition)
 * \brief Start GPS acquisition.
 * \param[in]   acquisition: Pointer to the GPS acquisition parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_start_acquisition(MAXM10X_acquisition_t* acquisition);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_stop_acquisition(void)
 * \brief Stop GPS acquisition.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_stop_acquisition(void);

/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_process(void)
 * \brief MAXM10X driver process function.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_process(void);

#ifdef MAXM10X_DRIVER_GPS_DATA_TIME
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_get_time(MAXM10X_time_t* gps_time)
 * \brief Read GPS time data of last acquisition.
 * \param[in]   none
 * \param[out]  gps_time: Pointer to the last GPS time data.
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_get_time(MAXM10X_time_t* gps_time);
#endif

#ifdef MAXM10X_DRIVER_GPS_DATA_POSITION
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_get_position(MAXM10X_position_t* gps_position)
 * \brief Read GPS position data of last acquisition.
 * \param[in]   none
 * \param[out]  gps_position: Pointer to the last GPS position data.
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_get_position(MAXM10X_position_t* gps_position);
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_set_backup_voltage(uint8_t state)
 * \brief Set GPS backup voltage state.
 * \param[in]   state: 0 to turn off, turn on otherwise.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_set_backup_voltage(uint8_t state);
#endif

#ifdef MAXM10X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_get_backup_voltage(uint8_t* state)
 * \brief Get GPS backup voltage state.
 * \param[in]   none
 * \param[out]  state: Pointer to the current VBCKP pin state.
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_get_backup_voltage(uint8_t* state);
#endif

#ifdef MAXM10X_DRIVER_TIMEPULSE
/*!******************************************************************
 * \fn MAXM10X_status_t MAXM10X_set_timepulse(MAXM10X_timepulse_configuration_t* configuration)
 * \brief Configure GPS timepulse output.
 * \param[in]   configuration: Pointer to the timepulse signal parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MAXM10X_status_t MAXM10X_set_timepulse(MAXM10X_timepulse_configuration_t* configuration);
#endif

/*******************************************************************/
#define MAXM10X_exit_error(base) { ERROR_check_exit(maxm10x_status, MAXM10X_SUCCESS, base) }

/*******************************************************************/
#define MAXM10X_stack_error(base) { ERROR_check_stack(maxm10x_status, MAXM10X_SUCCESS, base) }

/*******************************************************************/
#define MAXM10X_stack_exit_error(base, code) { ERROR_check_stack_exit(maxm10x_status, MAXM10X_SUCCESS, base, code) }

#endif /* MAXM10X_DRIVER_DISABLE */

#endif /* __MAXM10X_H__ */
