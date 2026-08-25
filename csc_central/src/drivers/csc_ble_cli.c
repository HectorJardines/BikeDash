#include "../../include/drivers/csc_ble_cli.h"
#include "../../include/common/defines.h"
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(csc_cli, LOG_LEVEL_DBG);

/********************
 * MACROS / STRUCTS
 *******************/
#define MAX_CONNS   (2U)
#define GET_CADENCE_CONN(connections)      (connections[0].type == CSC_SENSOR_MODE_CADENCE ? &connections[0] : connections[1].type == CSC_SENSOR_MODE_CADENCE ? &connections[1] : NULL)
#define GET_SPEED_CONN(connections)        (connections[0].type == CSC_SENSOR_MODE_SPEED ? &connections[0] : connections[1].type == CSC_SENSOR_MODE_SPEED ? &connections[1] : NULL)

#define BLE_CONN_INT_MIN    (BT_GAP_MS_TO_CONN_INTERVAL(225U)) // in ms
#define BLE_CONN_INT_MAX    (BT_GAP_MS_TO_CONN_INTERVAL(325U)) // in ms
#define BLE_CONN_TIMEOUT    (BT_GAP_MS_TO_CONN_TIMEOUT(4000U))
#define BLE_CONN_LATENCY    (4U)   // in intervals (allows peripheral to skipp 4 connection intervals if no data)


struct csc_dev_conn_inf {
    uint8_t type;

    struct bt_conn *p_conn;
    struct bt_uuid_16 disc_uuid;
    struct bt_gatt_discover_params disc_params;

    struct bt_gatt_subscribe_params sc_cp_params;
    struct bt_gatt_subscribe_params meas_params;
    uint16_t sensor_loc_handle;
    uint16_t csc_feature_handle;
};


/**********************
 * STATIC DECLARATIONS
 ************************/
static uint8_t cli_scan_csc_server(void);
static void cli_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad);
static void cli_scan_timeout(void);
static void on_connected(struct bt_conn *conn, uint8_t conn_err);
static void on_disconnected(struct bt_conn *conn, uint8_t reason);
static uint8_t cli_notify_handle(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length);
static uint8_t cli_discover_cb(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params);
static uint8_t cli_gatt_read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params, const void *data, uint16_t length);

static struct csc_dev_conn_inf connections[MAX_CONNS];
#define GET_CONN_STRUCT(conn)       (conn == connections[0].p_conn ? &connections[0] : conn == connections[1].p_conn ? &connections[1] : NULL)

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected      = on_connected,
    .disconnected   = on_disconnected
};

static struct bt_le_scan_cb scan_cbs = {
    .timeout        = cli_scan_timeout
};
/*******************
 * PUBLIC APIs
 *******************/



/**
 * @brief Initialize the BLE stack for the central device
 * 
 * 
 */
int csc_client_init(void) {
    int ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("BT CENTRAL FAILED TO ENABLE: %d\n\r", ret);
        return ret;
    }

    ret = bt_le_scan_cb_register(&scan_cbs);
    if (ret)
        LOG_ERR("FAILED TO REGISTER SCAN CALLBACKS\n\r");

    return ret;
}


/**
 * @brief Client scan for CSC sensor
 * 
 * 
 */
int csc_client_scan(void) {
    if (connections[0].p_conn == NULL || connections[1].p_conn == NULL)
        cli_scan_csc_server();
    else
        LOG_WRN("ALL CONNECTIONS IN USE CANNOT START SCAN\n\r");
}



/**
 * @brief Client disconnect from CSC sensor
 * 
 * 
 * 
 */
int csc_client_disconnect(void) {

}




/**
 * @brief 
 * 
 * 
 */
int cli_req_sc_cp_op(uint8_t sc_cp_op, void *data, uint8_t sens_type) {
    int ret;
    struct sc_ctrl_point_write_req op_req;
    struct bt_gatt_write_params write_req;
    struct csc_dev_conn_inf *p_dev;
    if (sens_type == CSC_SENSOR_MODE_CADENCE)
        p_dev = GET_CADENCE_CONN(connections);
    else
        p_dev = GET_SPEED_CONN(connections);
    
    if (p_dev == NULL || p_dev->p_conn == NULL) {
        LOG_WRN("SENSOR TYPE NOT CONNECTED: %d\n\r", sens_type);
        return 1;
    }

    op_req.op = sc_cp_op;

    switch (sc_cp_op) {
    case SC_CP_OP_SET_CWR:
        op_req.cwr = *((uint32_t *)data);
        write_req.handle = p_dev->sc_cp_params.value_handle;
        write_req.length = sizeof(op_req.op) + sizeof(op_req.cwr);
        break;
    case SC_CP_OP_UPDATE_LOC:
        op_req.sens_loc = *((uint8_t *)data);
        write_req.handle = p_dev->sc_cp_params.value_handle;
        write_req.length = sizeof(op_req.op) + sizeof(op_req.sens_loc);
        break;
    case SC_CP_OP_REQ_SUPP_LOC:
    default:
        return 1; // NOT SUPPORTED RIGHT NOW
    }

    memcpy((void *)write_req.data, (const void *)&op_req, sizeof(op_req));
    write_req.func = NULL;

    ret = bt_gatt_write(p_dev->p_conn, &write_req);
    if (ret)
        LOG_ERR("FAILED TO WRITE TO SC CP CHRC\n\r");

    return ret;
}



/*******************
 * STATIC DEFS
 ********************/

static uint8_t cli_scan_csc_server(void) {
    int err;

    struct bt_le_scan_param scan_params = {
        .type           = BT_LE_SCAN_TYPE_ACTIVE,
        .interval       = BT_GAP_SCAN_FAST_INTERVAL,
        .window         = BT_GAP_SCAN_FAST_WINDOW,
        .timeout        = BT_GAP_MS_TO_CONN_TIMEOUT(30000), // 30 seconds to match peripheral ADV time
    };

    err = bt_le_scan_start(&scan_params, cli_device_found);
    if (err) {
        LOG_ERR("CENTRAL FAILED TO START LOG DEBUGGING\n\r");
        return err;
    }

    LOG_DBG("SCAN SUCCESFULLY STARTED\n\r");
    return err;
}


/**
 * @brief Callback function for parsing advertising data
 * 
 * This functions parses the advertising data from a 
 * scanned device. This function creates the BLE connection if 
 * the expected data is found.
 * 
 */
static bool ad_found(struct bt_data *data, void *user_data) {
    int err;
    bool cont = true;
    switch (data->type) {
    case BT_DATA_FLAGS:
        break;
    case BT_DATA_UUID16_ALL:
        cont = false;
        break;
    default:
        break;
    }

    if (cont == false) {
        uint8_t csc_found = 0;
        struct bt_conn_le_create_param *create_param;
        struct bt_le_conn_param *param;
        const struct bt_uuid *uuid;
        for (uint8_t i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			uint16_t u16;
			
			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_CSC))
				continue;
            csc_found = 1;
            break;
        }

        if (!csc_found)
            LOG_DBG("ADV PACKET DID NOT CONTAIN CSC UUID\n\r");
        else {
            err = bt_le_scan_stop();
            if (err) {
                LOG_ERR("FAILED TO STOP BT LE SCAN\n\r");
                cont = true;
                return cont;
            }
            param = BT_LE_CONN_PARAM(BLE_CONN_INT_MIN, BLE_CONN_INT_MAX, BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);
            // THESE PARAMS SHOULD BE CHANGED AS WELL NEED TO READ UP ON THEM
            create_param = BT_CONN_LE_CREATE_CONN;

            struct bt_conn **p_conn = connections[0].p_conn == NULL ? &connections[0].p_conn : &connections[1].p_conn;
            err = bt_conn_le_create((bt_addr_le_t *)user_data, create_param, param, p_conn);
            if (err)
                LOG_ERR("CONN FAILED TO ESTABLISH\n\r");
        }
    }

    return cont;
}

static void cli_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
    LOG_INF("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       bt_addr_le_str(addr), type, ad->len, rssi);
    
    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
	    type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND ||
	    type == BT_GAP_ADV_TYPE_EXT_ADV) {
        bt_data_parse(ad, ad_found, (void *)addr);
    }
}


static void cli_scan_timeout(void) {

}


static void on_connected(struct bt_conn *conn, uint8_t conn_err) {
    int ret; 

    if (conn_err) {
        LOG_ERR("CONN FAILED TO ESTABLISH\n\r");

        struct bt_conn **p_conn = conn == connections[0].p_conn ? &connections[0].p_conn : conn == connections[1].p_conn ? &connections[1].p_conn : NULL; 
        if (p_conn != NULL) {
            bt_conn_unref(*p_conn);
            *p_conn = NULL;
        }
        return;
    }


    LOG_INF("CONNECTION ESTABLISHED\n\r");

    if (conn == connections[0].p_conn || conn == connections[1].p_conn) {
        struct csc_dev_conn_inf *p_dev = NULL;
        if (conn == connections[0].p_conn)
            p_dev = &connections[0];
        else 
            p_dev = &connections[1];


        memcpy((void *)&p_dev->disc_uuid, (const void *)BT_UUID_CSC, sizeof(p_dev->disc_uuid));
        p_dev->disc_params.uuid = &p_dev->disc_uuid.uuid;
        p_dev->disc_params.func = cli_discover_cb;
        p_dev->disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
        p_dev->disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        p_dev->disc_params.type = BT_GATT_DISCOVER_PRIMARY;

        ret = bt_gatt_discover(p_dev->p_conn, &p_dev->disc_params);

        if (ret) {
            LOG_ERR("FAILED TO BEGIN SERVICE DISCOVERY PHASE\n\r");
            return;
        }
    }
}



static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn == connections[0].p_conn) {
        bt_conn_unref(connections[0].p_conn);
        connections[0].p_conn = NULL;
    } else if (conn == connections[1].p_conn) {
        bt_conn_unref(connections[1].p_conn);
        connections[1].p_conn = NULL;
    }

    LOG_DBG("Connection terminated: %d\n\r", reason);
    return;
}



static uint8_t cli_notify_handle(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
    int ret = 0;
    if (length != sizeof(struct csc_meas_notify)) {
        LOG_WRN("RX NOTIFY DATA LEN DOES NOT MATCH EXPECTED\n\r");
        return 1;
    }

    const struct csc_meas_notify *nfy = ((const struct csc_meas_notify *)data);
    extern int (*comp_post_event)(uint32_t, struct csc_meas_notify *);
    if (nfy != NULL) {
        ret = comp_post_event(EVT_BLE_NFY_Msk, nfy);
        if (ret)
            LOG_ERR("FAILED TO POST BLE NOTIFY EVENT\n\r");
    }

    return ret;
}



static uint8_t cli_indicate_handle(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
    // the BLE stack handles sending ACK back to the peripheral 
    struct sc_ctrl_point_indicate *ind;
    ind = (struct sc_ctrl_point_indicate *)data;

    if (ind->op != SC_CP_OP_RESPONSE) {
        LOG_ERR("CLIENT INDICATE ONLY EXPECTS SC CP RESPONSE OPERATION\n\r");
        return 1;
    }

    if (ind->status != SC_CP_RSP_SUCCESS) {
        LOG_DBG("CLIENT OPERATION REQ: %d, FAILED\n\r", ind->req_op);
        return 1;
    }

    switch (ind->req_op) {
    case SC_CP_OP_SET_CWR:
        LOG_INF("SUCCESSFULLY RESET CWR\n\r");
        break;
    case SC_CP_OP_UPDATE_LOC:
        LOG_INF("SUCCESSFULLY UPDATED SENSOR LOCATION\n\r");
        break;
    case SC_CP_OP_REQ_SUPP_LOC:
    default:
        break;
    }

    return 0;
}



static uint8_t discover_param_configure(struct csc_dev_conn_inf *p_dev, const struct bt_gatt_attr *attr, 
                                    const struct bt_uuid *uuid, uint8_t uuid_type, uint8_t offset) {
    uint8_t ret = 0;
    memcpy((void *)&p_dev->disc_uuid, (const void *)BT_UUID_CSC_MEASUREMENT, sizeof(p_dev->disc_uuid));
    p_dev->disc_params.uuid = &p_dev->disc_uuid.uuid;
    p_dev->disc_params.start_handle = attr->handle + offset;
    p_dev->disc_params.type = uuid_type;

    ret = bt_gatt_discover(p_dev->p_conn, &p_dev->disc_params);
    if (ret)
        __NOP();

    return ret;
}


/**
 * @brief Discover relevant characteristics of the CSC service
 * 
 * This callback funciton discovers the characteristics/descriptors of a 
 * service. It also stores the relevant value and CCC handles for 
 * later use.
 * 
 */
static uint8_t cli_discover_cb(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
    int ret;
    struct csc_dev_conn_inf *p_dev = GET_CONN_STRUCT(conn);
    
    if (!attr) {
        LOG_INF("Discover Phase Complete\n\r");
        memset((void *)&p_dev->disc_params, 0, sizeof(p_dev->disc_params));
        return BT_GATT_ITER_STOP;
    }

    if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_CSC)) {
        discover_param_configure(p_dev, attr, BT_UUID_CSC_MEASUREMENT, BT_GATT_DISCOVER_CHARACTERISTIC, 1);
    } else if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_CSC_MEASUREMENT)) {
        discover_param_configure(p_dev, attr, BT_UUID_GATT_CCC, BT_GATT_DISCOVER_DESCRIPTOR, 2);
    } else if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_GATT_CCC)) {
        p_dev->meas_params.value = BT_GATT_CCC_NOTIFY;
        p_dev->meas_params.notify = cli_notify_handle;
        p_dev->meas_params.ccc_handle = attr->handle;

        ret = bt_gatt_subscribe(p_dev->p_conn, &p_dev->meas_params);
        if (ret)
            LOG_ERR("FAILED TO SUB TO NOTIFICATION\n\r");
        else
            LOG_DBG("SUBSCRBED TO NOTIFICATIONS\n\r");
        discover_param_configure(p_dev, attr, BT_UUID_SENSOR_LOCATION, BT_GATT_DISCOVER_CHARACTERISTIC, 1);
    } else if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_SENSOR_LOCATION)) {
        p_dev->sensor_loc_handle = bt_gatt_attr_value_handle(attr);
        discover_param_configure(p_dev, attr, BT_UUID_CSC_FEATURE, BT_GATT_DISCOVER_CHARACTERISTIC, 1);
    } else if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_CSC_FEATURE)) {
        p_dev->csc_feature_handle = bt_gatt_attr_value_handle(attr);
        discover_param_configure(p_dev, attr, BT_UUID_SC_CONTROL_POINT, BT_GATT_DISCOVER_CHARACTERISTIC, 1);
    } else if (!bt_uuid_cmp(p_dev->disc_params.uuid, BT_UUID_SC_CONTROL_POINT)) {
        p_dev->sc_cp_params.value_handle = bt_gatt_attr_value_handle(attr);
        discover_param_configure(p_dev, attr, BT_UUID_GATT_CCC, BT_GATT_DISCOVER_DESCRIPTOR, 2);
    } else {
        p_dev->sc_cp_params.value = BT_GATT_CCC_INDICATE;
        p_dev->sc_cp_params.notify = cli_indicate_handle;
        p_dev->sc_cp_params.ccc_handle = attr->handle;
        
        ret = bt_gatt_subscribe(p_dev->p_conn, &p_dev->sc_cp_params);
        if (ret)
            LOG_ERR("FAILED TO SUB TO INDICATIONS\n\r");
        else
            LOG_DBG("SUBSCRBED TO INDICATIONS\n\r");

        struct bt_gatt_read_params read_p;
        read_p.handle_count = 1;
        read_p.func = cli_gatt_read_cb;
        read_p.single.handle = p_dev->csc_feature_handle;
        read_p.single.handle = 0;
        ret = bt_gatt_read(p_dev->p_conn, &read_p);

        if (ret)
            LOG_WRN("FAILED TO READ CSC FEATURE ATTRIBUTE\n\r");
    }

    return BT_GATT_ITER_STOP;
}


static uint8_t cli_gatt_read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params, const void *data, uint16_t length) {
    uint32_t conn_csc_feats = *((uint32_t *)data);
    uint8_t idx = conn == connections[0].p_conn ? 0 : conn == connections[1].p_conn ? 1 : 0xFF;

    if (idx == 0xFF) {
        LOG_ERR("UNKNOWN CONNECTION IN READ CB\n\r");
        return 1;
    }

    if (conn_csc_feats & CSC_FEAT_WHEEL_REV)
        connections[idx].type = CSC_SENSOR_MODE_SPEED;
    else if (conn_csc_feats)
        connections[idx].type = CSC_SENSOR_MODE_CADENCE;

    return 0;
}


