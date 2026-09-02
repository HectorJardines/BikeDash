#include "../../include/drivers/csc_ble.h"
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(csc_ble, LOG_LEVEL_DBG);

/***********************
 * MACROS/TYPEDEFS
 ***********************/
#define IND_STRUCT_LEN      (3U)
#define MAX_IND_DATA_LEN    (5U)
#define IND_BUF_LEN         (IND_STRUCT_LEN + MAX_IND_DATA_LEN)

#define IND_ONGOING (1U)
#define IND_READY   (0U)

#define PERIPH_CADENCE_MODE (0U)
#define PERIPH_SPEED_MODE   (1U)

typedef struct {
    struct bt_gatt_indicate_params ind_params;
    uint8_t ind_params_data_buf[MAX_IND_DATA_LEN];
    uint8_t ind_ongoing;
} csc_ind_handle_t;

static sens_conn_cb csc_sens_conn_cb;
static sens_disconn_cb csc_sens_disconn_cb;
static volatile int nfy_cnt = 0;

#define SIGNAL_CONN_EST()   csc_sens_conn_cb()
#define SIGNAL_CONN_TERM()  csc_sens_disconn_cb()

/*************************
 * STATIC DECLARATIONS
 ************************/
static void csc_meas_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void ctrl_point_indicate(struct bt_conn *conn, uint8_t req_op, uint8_t status,
                                const void *data, uint16_t data_len);
static void ctrl_point_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void csc_cp_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err);

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t read_location(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                            uint16_t len, uint16_t offset);
static ssize_t read_csc_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset);

static void on_connected(struct bt_conn *conn, uint8_t err);
static void on_disconnected(struct bt_conn *conn, uint8_t reason);
static void on_recycled(void);
static int bt_ready(void);


static const struct bt_data scan_data[] = { // scan response packets
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1)
};
static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)), // NO BT CLASSIC SUPPORT + GENERAL ADV
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CSC_VAL)), // ADV CSC SERVICE
};


static struct csc_sensor_info_t sens;
static struct bt_le_adv_param *adv_params = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, 
                                            800,
                                            1000,
                                            NULL);
static csc_ind_handle_t h_ind;      
static struct bt_conn_cb conn_cbs = {
    .connected          = on_connected,
    .disconnected       = on_disconnected,
};
/*********************
 * PUBLIC APIs
 *********************/
BT_GATT_SERVICE_DEFINE(csc_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CSC),
    BT_GATT_CHARACTERISTIC(BT_UUID_CSC_MEASUREMENT, BT_GATT_CHRC_NOTIFY, 
                            0x00, NULL, NULL, NULL), // CSC MEASUREMENT NTFY CHRC
    BT_GATT_CCC(csc_meas_ccc_cfg_changed, (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION, BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ, read_location, NULL, &sens.sensor_loc), // CSC SENSOR LOC CHRC
    BT_GATT_CHARACTERISTIC(BT_UUID_CSC_FEATURE, BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ, read_csc_feature, NULL, NULL), // CSC SUPPORTED FEATURES CHRC
    BT_GATT_CHARACTERISTIC(BT_UUID_SC_CONTROL_POINT, (BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE),
                            BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &sens.sensor_loc),
    BT_GATT_CCC(ctrl_point_ccc_cfg_changed, (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
);


/**
 * @brief Enables and configures the BT stack
 * 
 * 
 */
int csc_ble_init(void) {
    int err = bt_enable(NULL);
    if (err) {
        printk("ERROR ENABLING BT PERIPHERAL: %d\n\r", err); 
        return err;
    }
    err = bt_conn_cb_register(&conn_cbs);
    if (err) {
        printk("ERROR SETTING CONN CBS\n\r"); 
        return err;
    }

    return err;
}


/**
 * @brief Starts BLE peripheral advertising with timeout
 * 
 * This function begins the BLE peripheral ADV phase, if no
 * connection is established within 30 seconds timeout and 
 * start will need to be triggered again
 * 
 */
int csc_ble_start_adv(void) {
    int32_t ret = bt_ready();
    return ret;
}


/**
 * @brief
 */
int csc_ble_measurement_notify(uint32_t *cwr, uint16_t *lwet, uint32_t *ccr, uint16_t *lcet) {
    if (!sens.meas_notify_configured)
        return 1;
    struct csc_meas_notify *meas_nfy;
    uint8_t buf[sizeof(*meas_nfy) + 
                (cwr != NULL ? sizeof(struct wheel_rev_notify) : 0) + 
                (ccr != NULL ? sizeof(struct crank_rev_notify) : 0)];
    uint16_t len = 0U;

    meas_nfy = (void *)buf;
    meas_nfy->flags = 0x00;

    if (cwr) {
        meas_nfy->flags |= CSC_WHEEL_REV_DATA_PRESENT;
        struct wheel_rev_notify wr_nfy;
        wr_nfy.cumulative_wr = sys_cpu_to_le32(*cwr); // expects little endian data
        wr_nfy.last_wet = sys_cpu_to_le16(*lwet);

        memcpy((void *)meas_nfy->data, (const void *)&wr_nfy, sizeof(wr_nfy));
        len += sizeof(wr_nfy);
    }

    if (ccr) {
        meas_nfy->flags |= CSC_CRANK_REV_DATA_PRESENT;
        struct crank_rev_notify cr_nfy;
        cr_nfy.cumulative_cr = sys_cpu_to_le32(*ccr);
        cr_nfy.last_cet = sys_cpu_to_le16(*lcet);

        memcpy((void *)(meas_nfy->data + len), (const void *)&cr_nfy, sizeof(cr_nfy));
    }

    int ret = bt_gatt_notify(sens.sens_conn, &csc_svc.attrs[1], buf, sizeof(buf));
    if (!ret) {
        nfy_cnt++;
    }
}


/**
 * @brief Check whether sensor BLE conn has been established
 * 
 * This function should be called before any calls to 
 * csc_ble_measurement_notify.
 * 
 * @return 1 if connection has been established; else 0
 */
void csc_ble_conn_cb_reg(sens_conn_cb conn, sens_disconn_cb disconn) {
    if (conn == NULL || disconn == NULL)
        LOG_WRN("NO CONNECTION CB REGISTERD\n\r");
    csc_sens_conn_cb = conn;
    csc_sens_disconn_cb = disconn;
}

/******************
 * STATIC DEFS
 ********************/

/* NOTIFICATION/INDICATION CHARACTERISTICS */

/**
 * @brief Updates the central's notify sub status
 * 
 * 
 */
static void csc_meas_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    sens.meas_notify_configured = value == BT_GATT_CCC_NOTIFY;
}


/**
 * @brief
 */
static void ctrl_point_indicate(struct bt_conn *conn, uint8_t req_op, uint8_t status,
                                const void *data, uint16_t data_len) 
{
    if (!sens.ctrl_point_configured) {
        LOG_ERR("INDICATE NOT CONF\n\r");
        return;
    }

    if (h_ind.ind_ongoing == IND_ONGOING) {
        LOG_WRN("INDICATE ONGOING, CANNOT START ANOTHER INDICATE EVENT\n\r");
        return;
    }

    struct sc_ctrl_point_indicate *ind;
    memset((void *)h_ind.ind_params_data_buf, 0, IND_BUF_LEN);

    ind = (void *)h_ind.ind_params_data_buf;
    ind->op = SC_CP_OP_RESPONSE;
    ind->req_op = req_op;
    ind->status = status;
    
    if (data && data_len > 0)
        memcpy((void *)ind->data, data, data_len);
    
    h_ind.ind_params.attr = &csc_svc.attrs[8];
    h_ind.ind_params.len = IND_STRUCT_LEN + data_len;
    h_ind.ind_params.data = h_ind.ind_params_data_buf;
    h_ind.ind_params.func = csc_cp_indicate_cb;
    h_ind.ind_params.destroy = NULL;

    bt_gatt_indicate(conn, &h_ind.ind_params); // NOTIFY BUT WE SET INDICATE?
}

/**
 * @brief Updates the central's indicate subscription (?) status
 * 
 * 
 * 
 */
static void ctrl_point_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    sens.ctrl_point_configured = value == BT_GATT_CCC_INDICATE;
}


static void csc_cp_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err) {
    if (err)
        LOG_ERR("ERROR IN INDICATE EVENT: %d\n\r", err);
    else
        LOG_DBG("SUCCESFUL INDICATION COMPLETE\n\r");
    h_ind.ind_ongoing = IND_READY;
}



/* BLE READ/WRITE CHARACTERISTICS */


/**
 * @brief
 */
static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                uint16_t len, uint16_t offset, uint8_t flags) 
{
    struct sc_ctrl_point_write_req *req = buf;
    uint8_t status;
    int i;

    if (!sens.ctrl_point_configured)
        return BT_GATT_ERR(CSC_ERR_CCC_CONFIG);

    if (!len)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    
    switch (req->op) {
    case SC_CP_OP_SET_CWR:
        if (len != sizeof(req->op) + sizeof(req->cwr)) {
            status = SC_CP_RSP_INVAL_PARAM;
            break;
        }

        sens.c_wheel_revs = sys_le32_to_cpu(req->cwr);
        status = SC_CP_RSP_SUCCESS;
        break;
    case SC_CP_OP_REQ_SUPP_LOC:
        if (len != sizeof(req->op)) {
            status = SC_CP_RSP_INVAL_PARAM;
            break;
        }

        ctrl_point_indicate(conn, req->op, SC_CP_RSP_SUCCESS, 
                            (const void *)&sens.supported_locs, ARRAY_SIZE(sens.supported_locs));
        return len;
    case SC_CP_OP_UPDATE_LOC:
        if (req->sens_loc == sens.sensor_loc) {
            status = SC_CP_RSP_SUCCESS;
            break;
        }
        status = SC_CP_RSP_INVAL_PARAM;

        for (i = 0; i < ARRAY_SIZE(sens.supported_locs); ++i) {
            if (sens.supported_locs[i] == req->sens_loc) {
                sens.sensor_loc = req->sens_loc;
                status = SC_CP_RSP_SUCCESS;
                break;
            }
        }
        break;
    default:
        status = SC_CP_RSP_OP_NOT_SUPP;
        break;
    }
     

    ctrl_point_indicate(conn, req->op, status, NULL, 0);
    return len;
}

/**
 * @brief
 */
static ssize_t read_location(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                            uint16_t len, uint16_t offset)
{
    uint8_t *value = attr->user_data; // stored sensor loc
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value)); // just memcpy value to correct offset in buf
}

/**
 * @brief
 */
static ssize_t read_csc_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset)
{
    uint16_t csc_feat;
    extern uint8_t(*accel_get_mode)(void);
    if (accel_get_mode() == PERIPH_CADENCE_MODE)
        csc_feat = CSC_FEATURES_CR;
    else
        csc_feat = CSC_FEATURES_WR;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &csc_feat, sizeof(csc_feat));
}



/* BLE SETUP FUNCTIONS */


/**
 * @brief
 */
static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("CONNECTION FAILED: %d\n\r", err); 
        return;
    }
    LOG_INF("CONNECTION ESTABLISHED\n\r");
    sens.sens_conn = bt_conn_ref(conn);
    SIGNAL_CONN_EST();
}



/**
 * @brief
 */
static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("BLE CONENCTION TERMINATED, REASON: %d\n\r", reason);
    bt_conn_unref(conn);
    sens.sens_conn = NULL;
    SIGNAL_CONN_TERM();
}


/**
 * @brief
 */
static int bt_ready(void) {
    int err = bt_le_adv_start(adv_params, adv_data, ARRAY_SIZE(adv_data), scan_data, ARRAY_SIZE(scan_data));
    if (err)
        LOG_ERR("ADV START FAILED: %d\n\r", err);
    else
        LOG_DBG("ADC START SUCCESSFUL\n\r");
    
    return err;
}


