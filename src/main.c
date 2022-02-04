#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>
#include <soc.h>
#include <drivers/pwm.h>
#include <logging/log.h>
#include <dk_buttons_and_leds.h>
#include <settings/settings.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>

/* Switch endpoint */
#define HA_SWITCH_ENDPOINT      12

/* Version of the application software (1 byte). */
#define SWITCH_INIT_BASIC_APP_VERSION     01

/* Version of the implementation of the Zigbee stack (1 byte). */
#define SWITCH_INIT_BASIC_STACK_VERSION   10

/* Version of the hardware of the device (1 byte). */
#define SWITCH_INIT_BASIC_HW_VERSION      11

/* Manufacturer name (32 bytes). */
#define SWITCH_INIT_BASIC_MANUF_NAME      "DongleNordic"

/* Model number assigned by manufacturer (32-bytes long string). */
#define SWITCH_INIT_BASIC_MODEL_ID        "DongleSwitch"

/* First 8 bytes specify the date of manufacturer of the device
 * in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific.
 */
#define SWITCH_INIT_BASIC_DATE_CODE       "202201207777"

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define SWITCH_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_BATTERY

/* Describes the physical location of the device (16 bytes).
 * May be modified during commisioning process.
 */
#define SWITCH_INIT_BASIC_LOCATION_DESC   "ManjuHome"

/* Describes the type of physical environment.
 * For possible values see section 3.2.2.2.10 of ZCL specification.
 */
#define SWITCH_INIT_BASIC_PH_ENV          ZB_ZCL_BASIC_ENV_UNSPECIFIED

#define TOGGLE_SWITCH					DK_BTN3_MSK

/* Nordic PWM nodes don't have flags cells in their specifiers, so
 * this is just future-proofing.
 */
#define FLAGS_OR_ZERO(node) \
	COND_CODE_1(DT_PHA_HAS_CELL(node, pwms, flags), \
		    (DT_PWMS_FLAGS(node)), (0))


#ifndef ZB_ED_ROLE
#error Define ZB_END_DEVICE_ROLE to compile enddevice source code.
#endif

// Manju - Declare the On_off_set_value function
static void on_off_set_value(zb_bool_t on, zb_uint8_t endpoint_invoked);
static void switch_send_on_off(zb_bufid_t bufid, zb_uint16_t on_off);
static void configure_reporting_locally();

LOG_MODULE_REGISTER(app);

/** 
 * Placeholder for the bulb context 
 * Stores all settings and static values.	
 **/
typedef struct 
{
	zb_zcl_basic_attrs_ext_t         basic_attr;
	zb_zcl_identify_attrs_t          identify_attr;
	zb_zcl_scenes_attrs_t            scenes_attr;
	zb_zcl_groups_attrs_t            groups_attr;
 	zb_zcl_on_off_attrs_t            on_off_attr;
} on_off_device_ctx_t;
/* Zigbee device application context storage. */
static on_off_device_ctx_t dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list,
	&dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
	scenes_attr_list,
	&dev_ctx.scenes_attr.scene_count,
	&dev_ctx.scenes_attr.current_scene,
	&dev_ctx.scenes_attr.current_group,
	&dev_ctx.scenes_attr.scene_valid,
	&dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

/* On/Off cluster attributes additions data */
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
	on_off_attr_list,
	&dev_ctx.on_off_attr.on_off);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CLUSTER_LIST(on_off_output_clusters,
    on_off_attr_list, basic_attr_list, identify_attr_list, groups_attr_list,
    scenes_attr_list);

ZB_HA_DECLARE_ON_OFF_OUTPUT_EP(on_off_output_ep, HA_SWITCH_ENDPOINT, on_off_output_clusters);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CTX(on_off_output_ctx, on_off_output_ep);

static void configure_reporting_locally(void)
{
  zb_zcl_reporting_info_t rep_info;
  zb_uint16_t coordinator_address = 0x0000;
  memset(&rep_info, 0, sizeof(rep_info));

  rep_info.direction      = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
  rep_info.ep             = HA_SWITCH_ENDPOINT;
  rep_info.cluster_id     = ZB_ZCL_CLUSTER_ID_ON_OFF;
  rep_info.cluster_role   = ZB_ZCL_CLUSTER_SERVER_ROLE;
  rep_info.attr_id        = ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;

  rep_info.u.send_info.min_interval = 0x000;
  rep_info.u.send_info.max_interval = 0x000;  
  rep_info.u.send_info.delta.u8     = 1;
   
  rep_info.dst.short_addr = coordinator_address; //Hex 
  rep_info.dst.endpoint   = 12 ;     //Decimal 
  rep_info.dst.profile_id = ZB_AF_HA_PROFILE_ID ;

  zb_zcl_put_reporting_info(&rep_info,ZB_TRUE);
}

void configure_attr_reporting(zb_bufid_t bufid)
{
    zb_uint8_t * cmd_ptr;
	zb_uint16_t coordinator_address = 0x0000;
    // zb_uint8_t dst_ep = HA_SWITCH_ENDPOINT;

    ZB_ZCL_GENERAL_INIT_CONFIGURE_REPORTING_SRV_REQ(bufid,
                                                    cmd_ptr,
                                                    ZB_ZCL_ENABLE_DEFAULT_RESPONSE);
 
    ZB_ZCL_GENERAL_ADD_SEND_REPORT_CONFIGURE_REPORTING_REQ(cmd_ptr, 
	                                                       ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
														   ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
														   0,
														   0, 
														   0);
 
    ZB_ZCL_GENERAL_SEND_CONFIGURE_REPORTING_REQ(bufid, cmd_ptr, coordinator_address, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                                HA_SWITCH_ENDPOINT, HA_SWITCH_ENDPOINT,
                                                ZB_AF_HA_PROFILE_ID, ZB_ZCL_CLUSTER_ID_ON_OFF, NULL);

	LOG_INF("Attribute Reporting configured");
}

/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing buttons state.
 * @param[in]   has_changed   Bitmask containing buttons
 *                            that have changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	zb_ret_t zb_err_code;
	zb_uint16_t cmd_id;

    user_input_indicate();
	/* Calculate bitmask of buttons that are pressed
	 * and have changed their state.
	 */
	uint32_t buttons = button_state & has_changed;

 	if (buttons & TOGGLE_SWITCH)  // Manju - Toggle the LED bulb by getting the context from the Zigbee object
	{
		// LOG_INF("Current LED setting is %d", dev_ctx.on_off_attr.on_off);

		if(dev_ctx.on_off_attr.on_off)
		{
			// on_off_set_value(ZB_FALSE, HA_BUTTON_ENDPOINT);
			cmd_id = ZB_ZCL_CMD_ON_OFF_OFF_ID;
		}
		else
		{
			// on_off_set_value(ZB_TRUE, HA_BUTTON_ENDPOINT);
			cmd_id = ZB_ZCL_CMD_ON_OFF_ON_ID;
		}
		// cmd_id = ZB_ZCL_CMD_ON_OFF_ON_ID;
		LOG_INF("Value update is %d", cmd_id);
				
		zb_err_code = zb_buf_get_out_delayed_ext(switch_send_on_off, cmd_id, 0);
		ZB_ERROR_CHECK(zb_err_code);

		LOG_INF("Toggling Completed");
	}
}

/**@brief Function for initializing LEDs and Buttons. */
static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
}

/**@brief Function for sending ON/OFF requests to the network / co-ordinator.
 *
 * @param[in]   bufid    Non-zero reference to Zigbee stack buffer that will be
 *                       used to construct on/off request.
 * @param[in]   cmd_id   ZCL command id.
 */
static void switch_send_on_off(zb_bufid_t bufid, zb_uint16_t cmd_id)
{
	LOG_INF("Send ON/OFF command: %d", cmd_id);
	zb_uint16_t coordinator_address = 0x0000;
	dev_ctx.on_off_attr.on_off = (zb_bool_t)cmd_id;	

	LOG_INF("Check Here:");

	ZB_ZCL_SET_ATTRIBUTE(
		HA_SWITCH_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&dev_ctx.on_off_attr.on_off,
		ZB_FALSE);
	
    // This method is to be used when 'commanOn' / 'commandOff is to be sent - especially for binded devices
	// ZB_ZCL_ON_OFF_SEND_REQ(bufid,
	// 			coordinator_address,
	// 			ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
	// 			HA_SWITCH_ENDPOINT,
	// 			HA_SWITCH_ENDPOINT,
	// 			ZB_AF_HA_PROFILE_ID,
	// 			ZB_ZCL_ENABLE_DEFAULT_RESPONSE,
	// 			cmd_id,
	// 			NULL);


	LOG_INF("Send ON/OFF command Completed: %d", cmd_id);
}

/**@brief Function for initializing all clusters attributes.
 */
static void switch_clusters_attr_init(void)
{
	/* Basic cluster attributes data */
	dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version   = SWITCH_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = SWITCH_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version    = SWITCH_INIT_BASIC_HW_VERSION;

	/* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte
	 * should contain string length without trailing zero.
	 *
	 * For example "test" string wil be encoded as:
	 *   [(0x4), 't', 'e', 's', 't']
	 */
	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		SWITCH_INIT_BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(SWITCH_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		SWITCH_INIT_BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(SWITCH_INIT_BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.date_code,
		SWITCH_INIT_BASIC_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(SWITCH_INIT_BASIC_DATE_CODE));

	dev_ctx.basic_attr.power_source = SWITCH_INIT_BASIC_POWER_SOURCE;

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.location_id,
		SWITCH_INIT_BASIC_LOCATION_DESC,
		ZB_ZCL_STRING_CONST_SIZE(SWITCH_INIT_BASIC_LOCATION_DESC));

	dev_ctx.basic_attr.ph_env = SWITCH_INIT_BASIC_PH_ENV;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off cluster attributes data. */
	dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;  // Manju - Original code. Only useful for the initial setting to OFF for the LED
	// dev_ctx.on_off_attr.on_off = ZB_FALSE;  // Manju - Setting this option sets the intial value to ON - which may not be desirable, hence the above original which starts the LED in OFF position to start with

	ZB_ZCL_SET_ATTRIBUTE(
		HA_SWITCH_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&dev_ctx.on_off_attr.on_off,
		ZB_FALSE);
	
	// zb_ret_t status = zb_zcl_start_attr_reporting(
	// 	HA_SWITCH_ENDPOINT, 
	// 	ZB_ZCL_CLUSTER_ID_ON_OFF, 
	// 	ZB_ZCL_CLUSTER_SERVER_ROLE, 
	// 	ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
	
	// if(status == RET_OK)
	// {
	// 	LOG_INF("Reporting successfully configured");
	// }
	// else
	// {
	// 	LOG_INF("Reporting IS NOT configured");
	// }
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_uint8_t cluster_id;
	zb_uint8_t attr_id;
	zb_uint8_t endpoint_invoked;	
	zb_zcl_device_callback_param_t  *device_cb_param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);
	
	endpoint_invoked = device_cb_param->endpoint;

	LOG_INF("%s id %hd", __func__, device_cb_param->device_cb_id);

	LOG_INF("EndPoint Called is %d ", endpoint_invoked);

	/* Set default response value. */
	device_cb_param->status = RET_OK;
	LOG_INF("ZB Callback started %c", bufid);	

	// switch (device_cb_param->device_cb_id) 
	// {
	// 	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
	// 		LOG_INF("Inside the ZB_ZCL_SET_ATTR_VALUE_CB_ID");
	// 		cluster_id = device_cb_param->cb_param.set_attr_value_param.cluster_id;
	// 		attr_id = device_cb_param->cb_param.set_attr_value_param.attr_id;

	// 		if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
	// 		{
	// 			uint8_t value = device_cb_param->cb_param.set_attr_value_param.values.data8;

	// 			LOG_INF("on/off attribute setting to %hd", value);
	// 			if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) 
	// 			{
	// 				// on_off_set_value((zb_bool_t)value);
	// 				// Manju - call the on_off_set_value with the endpoint invoked information
	// 				//on_off_set_value((zb_bool_t)value, endpoint_invoked);
	// 				LOG_INF("ON INVOKED:");
	// 			}
	// 		} 
	// 		else 
	// 		{
	// 			/* Other clusters can be processed here */
	// 			LOG_INF("Unhandled cluster attribute id: %d",
	// 				cluster_id);
	// 		}
	// 		break;

	// 	default:
	// 		device_cb_param->status = RET_ERROR;
	// 		break;
	// }

	LOG_INF("%s status: %hd", __func__, device_cb_param->status);
}

void zboss_signal_handler(zb_bufid_t bufid) 
{
	zb_zdo_app_signal_hdr_t *p_sg_p = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &p_sg_p);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	/* Update network status LED */
	// zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

	switch (sig) 
	{
		case ZB_BDB_SIGNAL_DEVICE_REBOOT:
			/* fall-through */
		case ZB_BDB_SIGNAL_STEERING:
			/* Call default signal handler. */
			ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
			if (status == RET_OK) 
			{
				LOG_INF("Steering Completed. Going ahead with reporting configuration");
				// configure_reporting_locally();
				// zb_buf_get_out_delayed(configure_attr_reporting);
				// zb_zcl_start_attr_reporting(HA_SWITCH_ENDPOINT, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
			}
			break;
		default:
			/* Call default signal handler. */
			ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
			break;
	}

	if (bufid) 
	{
		zb_buf_free(bufid);
	}
}

void error(void)
{
	while (true) 
	{
		/* Spin forever */
		k_sleep(K_MSEC(1000));
	}
}

void main(void)
{
	int err;

	LOG_INF("Starting Manju Switch example");

	/* Initialize */
	configure_gpio();
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings initialization failed");
	}

	LOG_INF("Registering Device");
	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

	LOG_INF("Setting Switch Context");
	/* Register dimmer switch device context (all the endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&on_off_output_ctx);

	switch_clusters_attr_init();
	// configure_reporting_locally();

	/* Initialize ZCL scene table */
	// zcl_scenes_init();

	LOG_INF("ZCL scenes initiated");

	/* Settings should be loaded after zcl_scenes_init */
	err = settings_load();
	if (err) {
		LOG_ERR("settings loading failed");
	}

	/* Start Zigbee default thread */
	zigbee_enable();


	LOG_INF("Manju Switch example started");

	while (1) {
		// k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}