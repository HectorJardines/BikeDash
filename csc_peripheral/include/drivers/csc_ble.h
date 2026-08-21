#ifndef _BLE_H
#define _BLE_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

/****************
 * MACROS
 *****************/
#define CSC_SUPPORTED_LOCATIONS		{CSC_LOC_FRONT_WHEEL, \
                                    CSC_LOC_REAR_WHEEL, \
                                    CSC_LOC_LEFT_CRANK, \
                                    CSC_LOC_RIGHT_CRANK }

#define CSC_FEATURE			(CSC_FEAT_WHEEL_REV | \
					        CSC_FEAT_CRANK_REV | \
					        CSC_FEAT_MULTI_SENSORS)

/* CSC Sensor Locations */
#define CSC_LOC_OTHER			0x00
#define CSC_LOC_TOP_OF_SHOE		0x01
#define CSC_LOC_IN_SHOE			0x02
#define CSC_LOC_HIP			    0x03
#define CSC_LOC_FRONT_WHEEL		0x04
#define CSC_LOC_LEFT_CRANK		0x05
#define CSC_LOC_RIGHT_CRANK		0x06
#define CSC_LOC_LEFT_PEDAL		0x07
#define CSC_LOC_RIGHT_PEDAL		0x08
#define CSC_LOC_FRONT_HUB		0x09
#define CSC_LOC_REAR_DROPOUT	0x0a
#define CSC_LOC_CHAINSTAY		0x0b
#define CSC_LOC_REAR_WHEEL		0x0c
#define CSC_LOC_REAR_HUB		0x0d
#define CSC_LOC_CHEST			0x0e

/* CSC Application error codes */
#define CSC_ERR_IN_PROGRESS		0x80
#define CSC_ERR_CCC_CONFIG		0x81

/* SC Control Point Opcodes */
#define SC_CP_OP_SET_CWR		    0x01
#define SC_CP_OP_CALIBRATION		0x02 // NOT USED
#define SC_CP_OP_UPDATE_LOC		    0x03
#define SC_CP_OP_REQ_SUPP_LOC		0x04
#define SC_CP_OP_RESPONSE		    0x10 // NOT USED

/* SC Control Point Response Values */
#define SC_CP_RSP_SUCCESS		    0x01
#define SC_CP_RSP_OP_NOT_SUPP		0x02
#define SC_CP_RSP_INVAL_PARAM		0x03
#define SC_CP_RSP_FAILED		    0x04

/* CSC Feature */
#define CSC_FEAT_WHEEL_REV		    BIT(0)
#define CSC_FEAT_CRANK_REV		    BIT(1)
#define CSC_FEAT_MULTI_SENSORS		BIT(2)

/* CSC Measurement Flags */
#define CSC_WHEEL_REV_DATA_PRESENT	BIT(0)
#define CSC_CRANK_REV_DATA_PRESENT	BIT(1)


struct csc_sensor_info_t {
    struct bt_conn *sens_conn;
    uint32_t c_wheel_revs;
    uint8_t supported_locs[4];
    uint8_t sensor_loc;
    uint8_t ctrl_point_configured;
    uint8_t meas_notify_configured;
};


struct csc_meas_notify {
    uint8_t flags;      /* type of data present */
    uint8_t data[];
}__packed;


struct wheel_rev_notify {
    uint32_t cumulative_wr;     /* CUMULATIVE WHEEL REVOLUTIONS */
    uint32_t last_wet;          /* LAST WHEEL EVENT TIME */
}__packed; /* wheel revolutions + last wheel event time*/


struct crank_rev_notify {
    uint32_t cumulative_cr;     /* CUMULATIVE CRANK REVOLUTIONS */
    uint32_t last_cet;          /* LAST CRANK EVENT TIME */
}__packed;


struct sc_ctrl_point_write_req {
    uint8_t op;
    union {
        uint32_t cwr;            /* CENTRAL SET CUMULATIVE WHEEL REV */
        uint8_t sens_loc;       /* CENTRAL UPDATE SENSOR LOCATION */
    };
}__packed;


struct sc_ctrl_point_indicate {
    uint8_t op;         /* SC CTRL POINT OPERATION ref @ SC Control Point Opcodes */
    uint8_t req_op;
    uint8_t status;     /* CTRL POINT OPERATION STATUS ref @ SC Control Point Response Values*/
    uint8_t data[];
}__packed;


/*****************
 * PUBLIC APIs
 ****************/


/**
 * @brief Initialize and configure the BLE stack and service information
 * 
 * 
 */
int csc_ble_init(void);



/**
 * @brief Starts BLE peripheral advertising with timeout
 * 
 * This function begins the BLE peripheral ADV phase, if no
 * connection is established within 30 seconds timeout and 
 * start will need to be triggered again
 * 
 */
void csc_ble_start_adv(void);


/**
 * @brief Notify central of wheel/crank events, e.g. increase in CWR/CCR
 * 
 * This function sends a BLE notification to the central device with either 
 * CWR + LWET OR CCR + LCET. The specific values will depend on the configuration
 * mode of the sensor e.g. cadence mode or speed mode.
 * 
 * @param cwr cumulative wheel revolution count
 * @param lwet last wheel event timestamp
 * @param ccr cumulative crank revolution count
 * @param lcet last crank event timestamp
 */
void csc_ble_measurement_notify(uint32_t cwr, uint16_t lwet, uint32_t ccr, uint16_t lcet);


/**
 * @brief Check whether sensor BLE conn has been established
 * 
 * This function should be called before any calls to 
 * csc_ble_measurement_notify.
 * 
 * @return 1 if connection has been established; else 0
 */
uint8_t csc_ble_is_connected(void);

#endif