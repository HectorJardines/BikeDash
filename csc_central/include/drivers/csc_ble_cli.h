#ifndef _CSC_CLI_H
#define _CSC_CLI_H

#include <stdint.h>

/*******************
 * MACROS
 ******************/
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

#define CSC_SENSOR_MODE_CADENCE (0U)
#define CSC_SENSOR_MODE_SPEED   (1U)



#define CLI_STATE_IDLE          (0U)
#define CLI_STATE_SCANNING      (1U)
#define CLI_STATE_CONNECTING    (2U)
/****************
 * STRUCTS/ENUMS
 ******************/
struct csc_meas_notify {
    uint8_t flags;      /* type of data present */
    uint8_t data[];
}__packed;


struct wheel_rev_notify {
    uint32_t cumulative_wr;     /* CUMULATIVE WHEEL REVOLUTIONS */
    uint16_t last_wet;          /* LAST WHEEL EVENT TIME */
}__packed; /* wheel revolutions + last wheel event time*/


struct crank_rev_notify {
    uint32_t cumulative_cr;     /* CUMULATIVE CRANK REVOLUTIONS */
    uint16_t last_cet;          /* LAST CRANK EVENT TIME */
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


struct csc_ble_cli_inf {
    uint8_t state;
    uint8_t connection_cnt;
};


/*******************
 * PUBLIC APIs
 *******************/



/**
 * @brief Initialize the BLE stack for the central device
 * 
 * 
 */
int csc_client_init(void);


/**
 * @brief Client scan for CSC sensor
 * 
 * 
 */
int csc_client_scan(void);



/**
 * @brief Client disconnect from CSC sensor
 * 
 * 
 * 
 */
int csc_client_disconnect(void);



int cli_req_sc_cp_op(uint8_t sc_cp_op, void *data, uint8_t sens_type);


#endif /* CSC_CLI_H */