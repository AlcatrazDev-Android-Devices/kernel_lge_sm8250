#define pr_fmt(fmt) "UNINODE: %s: " fmt, __func__
#define pr_uninode(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define pr_dbg_uninode(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#define UNIFIED_NODES_DEVICE	"lge-unified-nodes"
#define UNIFIED_NODES_DISABLED	-1000000

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>

#include "veneer-primitives.h"

#define VOTER_NAME_THERMALD	"THERMALD"
#define VOTER_NAME_HIDDENM	"HIDDENM"
#define RESTRICTION_MAX_COUNT	8
#define RESTRICTION_MAX_NAME	32

struct unified_nodes {
	struct device_node* devnode_restrictchg;

	struct voter_entry		thermald_iusb;
	struct voter_entry		thermald_ibat;
	struct voter_entry		thermald_idc;
	struct voter_entry		thermald_vdc;
	struct voter_entry		charging_restriction [RESTRICTION_MAX_COUNT];
	struct voter_entry		charging_enable [2]; // 0 for iusb, 1 for idc
	const char*			charging_step;
	bool				charging_showcase;
	void*				charging_completed;
	int				status_boot;
	bool				status_lcd;
	bool				fake_battery;
	bool				fake_sdpmax;
	bool				fake_hvdcp;
	void*				battery_age;
	void*				battery_condition;
	void*				battery_cycle;
	bool				battery_valid;
	char				charger_name [64];
	bool				charger_highspeed;
	bool*				charger_incompatible;
	void*				charger_usbid;
	enum charging_verbosity*	charger_verbose;
	// Supporting features : used for verifying device
	int				support_fastpl;
	bool				support_fastchg;
	const char* fake_psy;
	const char* battage_psy;
	int				fake_temperature;
	int				fake_mvoltage;
	int				fake_capacity;
	int				bsm_timetofull;
	struct thermal_zone_device *xo_tz;
	struct thermal_zone_device *bd_tz;

	struct __actm {
		int mode;
		int sensor_type[2];
		int lcdon_temp_offset;
		int max_hold_criteria_wired[3];
		int max_hold_criteria_wireless[3];
		int tempoffs_wired[3];
		int tempoffs_wireless[3];
		int power_epp[3];
		int power_bpp[3];
		int current_wired[3];
		int wired_max_fcc[3];
		int current_cp[2];
	} actm;
	int irc_enabled;
	int irc_resistance;

	int pps_ta_count;
};

static void voter_unregister(struct unified_nodes* uninodes) {
	int i;
	struct voter_entry* restriction;

	// For Restriction
	for (i = 0; i < RESTRICTION_MAX_COUNT; i++) {
		restriction = &uninodes->charging_restriction[i];
		if (restriction->type != VOTER_TYPE_INVALID)
			veneer_voter_unregister(restriction);
	}
	// For Thermald
	veneer_voter_unregister(&uninodes->thermald_iusb);
	veneer_voter_unregister(&uninodes->thermald_ibat);
	veneer_voter_unregister(&uninodes->thermald_idc);
	// For Hiddenm
	veneer_voter_unregister(&uninodes->charging_enable[0]); // 0 for iusb,
	veneer_voter_unregister(&uninodes->charging_enable[1]); // 1 for idc,
}

static bool voter_register(struct unified_nodes* uninodes) {
	bool ret = true;

// For Sysnode
	struct device_node* child;
	int i = 0;
	char* restrict_name;
	char* restrict_type;
	struct voter_entry* restrict_voter;

	for_each_child_of_node(uninodes->devnode_restrictchg, child) {
		if (i < RESTRICTION_MAX_COUNT) {
			restrict_voter = &uninodes->charging_restriction[i];

			if (ret && !of_property_read_string(child, "restrict-name", (const char **) &restrict_name)
				&& !of_property_read_string(child, "restrict-type", (const char **) &restrict_type)
				&& strlen(restrict_name) < RESTRICTION_MAX_NAME) {
				ret = ret && veneer_voter_register(restrict_voter, restrict_name, vote_fromdt(restrict_type), false);
			}
			else {
				pr_uninode("Parsing error on restricted charging nodes\n");
				return false;
			}
			i++;
		}
		else {
			pr_uninode("Overflow! Check the count of restrictions\n");
			return false;
		}
	}

	ret = ret
// For Thermald
	&& veneer_voter_register(&uninodes->thermald_iusb, VOTER_NAME_THERMALD, VOTER_TYPE_IUSB, false)
	&& veneer_voter_register(&uninodes->thermald_ibat, VOTER_NAME_THERMALD, VOTER_TYPE_IBAT, false)
	&& veneer_voter_register(&uninodes->thermald_idc, VOTER_NAME_THERMALD, VOTER_TYPE_IDC, false)
// For Hiddenm
	&& veneer_voter_register(&uninodes->charging_enable[0], VOTER_NAME_HIDDENM, VOTER_TYPE_IUSB, true)
	&& veneer_voter_register(&uninodes->charging_enable[1], VOTER_NAME_HIDDENM, VOTER_TYPE_IDC, true);

	if (!ret) {
		pr_uninode("failed to register voters\n");
	}

	return ret;
}

static ssize_t voter_store(struct voter_entry* voter, const char* buf, size_t size) {
	long limit;
	pr_uninode("%s(%s) writes %s\n", voter->name, vote_title(voter->type), buf);

	if ( !(kstrtol(buf, 10, &limit)) && (0<=limit) ) {
		// uninodes->voter_thermald_iusb->limit will be updated to limit_ma
		// after veneer_voter_set
		veneer_voter_set(voter, limit);
	}
	else
		veneer_voter_release(voter);

	return size;
}

static ssize_t voter_show(struct voter_entry* voter, char* buf) {
	return snprintf(buf, PAGE_SIZE, "%d", voter->limit);
}

static bool unified_nodes_actm_dt(
	struct device_node* devnode, struct unified_nodes* uninodes)
{
	struct device_node* dnode = devnode->parent;
	struct device_node* devnode_actm;
	struct device_node* devnode_bvp;

	int ret = 0;
	bool actm_enable = false;
	int actm_array_size = 0;
	int actm_array[3] = {0, };

	devnode_actm = of_find_node_by_name(dnode, "adaptive-charging-thermal");
	devnode_bvp = of_find_node_by_name(dnode, "protection-batvolt");

	uninodes->irc_enabled = of_property_read_bool(devnode_bvp, "lge,irc-enable");
	ret = of_property_read_s32(devnode_bvp,
			"lge,irc-resist-mohm", &uninodes->irc_resistance);
	if (ret != 0) {
		pr_uninode("Failed to read 'lge,irc-resist-mohm'(%d)\n", ret);
		uninodes->irc_resistance = 0;
	}

	actm_enable = of_property_read_bool(devnode_actm, "lge,actm-enable");
	if (actm_enable) {
		ret = of_property_read_s32(devnode_actm,
			"lge,actm-default-mode", &uninodes->actm.mode);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,actm-default-mode'(%d)\n", ret);
			uninodes->actm.mode = -2;
		}

		ret = of_property_read_s32(devnode_actm,
			"lge,actm-lcdon-temp-offset", &uninodes->actm.lcdon_temp_offset);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,actm-lcdon-temp-offset'(%d)\n", ret);
			uninodes->actm.lcdon_temp_offset = 0;
		}

		ret = of_property_read_u32(devnode_actm,
			"lge,wired-therm-sensor-type", &uninodes->actm.sensor_type[0]);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-therm-sensor-type'(%d)\n", ret);
			uninodes->actm.sensor_type[0] = 3;
		}

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wired-max-hold-criteria", actm_array, 3);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-max-hold-criteria'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.max_hold_criteria_wired[0] = actm_array[0];
		uninodes->actm.max_hold_criteria_wired[1] = actm_array[1];
		uninodes->actm.max_hold_criteria_wired[2] = actm_array[2];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wireless-max-hold-criteria", actm_array, 3);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-max-hold-criteria'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.max_hold_criteria_wireless[0] = actm_array[0];
		uninodes->actm.max_hold_criteria_wireless[1] = actm_array[1];
		uninodes->actm.max_hold_criteria_wireless[2] = actm_array[2];

		ret = of_property_read_u32(devnode_actm,
			"lge,wired-stage-size", &actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-stage-size'(%d)\n", ret);
			actm_array_size = 3;
		}

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wired-temp-offset", actm_array, actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-temp-offset'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.tempoffs_wired[0] = actm_array[0];
		uninodes->actm.tempoffs_wired[1] = actm_array[1];
		uninodes->actm.tempoffs_wired[2] = actm_array[2];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wired-current-limit-ma", actm_array, actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-current-limit-ma'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.current_wired[0] = actm_array[0];
		uninodes->actm.current_wired[1] = actm_array[1];
		uninodes->actm.current_wired[2] = actm_array[2];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wired-curr-cp-limit-ma", actm_array, 2);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-curr-cp-limit-ma'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
		}
		uninodes->actm.current_cp[0] = actm_array[0];
		uninodes->actm.current_cp[1] = actm_array[1];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wired-max-fcc-ma", actm_array, 3);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wired-max-fcc-ma'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.wired_max_fcc[0] = actm_array[0];
		uninodes->actm.wired_max_fcc[1] = actm_array[1];
		uninodes->actm.wired_max_fcc[2] = actm_array[2];

		ret = of_property_read_u32(devnode_actm,
			"lge,wireless-therm-sensor-type", &uninodes->actm.sensor_type[1]);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-therm-sensor-type'(%d)\n", ret);
			uninodes->actm.sensor_type[1] = 3;
		}

		ret = of_property_read_u32(devnode_actm,
			"lge,wireless-stage-size", &actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-stage-size'(%d)\n", ret);
			actm_array_size = 3;
		}

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wireless-temp-offset", actm_array, actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-epp-power-limit-mw'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.tempoffs_wireless[0] = actm_array[0];
		uninodes->actm.tempoffs_wireless[1] = actm_array[1];
		uninodes->actm.tempoffs_wireless[2] = actm_array[2];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wireless-epp-power-limit-mw", actm_array, actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-epp-power-limit-mw'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.power_epp[0] = actm_array[0];
		uninodes->actm.power_epp[1] = actm_array[1];
		uninodes->actm.power_epp[2] = actm_array[2];

		ret = of_property_read_u32_array(devnode_actm,
			"lge,wireless-bpp-power-limit-mw", actm_array, actm_array_size);
		if (ret != 0) {
			pr_uninode("Failed to read 'lge,wireless-bpp-power-limit-mw'(%d)\n", ret);
			actm_array[0] = 0;
			actm_array[1] = 0;
			actm_array[2] = 0;
		}
		uninodes->actm.power_bpp[0] = actm_array[0];
		uninodes->actm.power_bpp[1] = actm_array[1];
		uninodes->actm.power_bpp[2] = actm_array[2];
	}
	else {
		uninodes->actm.mode = -2;
		uninodes->actm.lcdon_temp_offset = 0;
		uninodes->actm.max_hold_criteria_wired[0] = 0;
		uninodes->actm.max_hold_criteria_wired[1] = 0;
		uninodes->actm.max_hold_criteria_wired[2] = 0;
		uninodes->actm.max_hold_criteria_wireless[0] = 0;
		uninodes->actm.max_hold_criteria_wireless[1] = 0;
		uninodes->actm.max_hold_criteria_wireless[2] = 0;
		uninodes->actm.tempoffs_wired[0] = 0;
		uninodes->actm.tempoffs_wired[1] = 0;
		uninodes->actm.tempoffs_wired[2] = 0;
		uninodes->actm.tempoffs_wireless[0] = 0;
		uninodes->actm.tempoffs_wireless[1] = 0;
		uninodes->actm.tempoffs_wireless[2] = 0;
		uninodes->actm.current_wired[0] = 0;
		uninodes->actm.current_wired[1] = 0;
		uninodes->actm.current_wired[2] = 0;
		uninodes->actm.current_cp[0] = 0;
		uninodes->actm.current_cp[1] = 0;
		uninodes->actm.power_epp[0] = 0;
		uninodes->actm.power_epp[1] = 0;
		uninodes->actm.power_epp[2] = 0;
		uninodes->actm.power_bpp[0] = 0;
		uninodes->actm.power_bpp[1] = 0;
		uninodes->actm.power_bpp[2] = 0;
	}

	return true;
}

static bool unified_nodes_devicetree(struct device_node* devnode, struct unified_nodes* uninodes)
{
	struct device_node* devnode_fakebatt = NULL;
	struct device_node* devnode_battage = NULL;
	struct device_node* devnode_thermal = NULL;

	const char* xo_therm_name = NULL;
	const char* bd_therm_name = NULL;
	int buf = 0;
	int ret = 0;

	bool result = true;

	uninodes->support_fastpl = (!of_property_read_u32(devnode, "lge,feature-charging-parallel", &buf)
		&& buf == 1) ? 0 : -1; /* if parallel charging is not supported, set it to '-1' */
	uninodes->support_fastchg = !of_property_read_u32(devnode, "lge,feature-charging-highspeed", &buf)
		? !!buf : 0;

	uninodes->devnode_restrictchg = of_find_node_by_name(devnode, "lge,restrict-charging");
	devnode_fakebatt = of_find_node_by_name(devnode, "lge,fake-battery");
	devnode_battage = of_find_node_by_name(devnode, "lge,battery-age");
	devnode_thermal = of_find_node_by_name(devnode, "lge,thermal-zone");

	ret = of_property_read_string(devnode_fakebatt, "fakebatt-psy", &uninodes->fake_psy);
	if (ret != 0) {
		pr_uninode("Failed to read 'fakebatt-psy'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_u32(devnode_fakebatt, "fakebatt-temperature", &uninodes->fake_temperature);
	if (ret != 0) {
		pr_uninode("Failed to read 'fakebatt-temperature'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_u32(devnode_fakebatt, "fakebatt-mvoltage", &uninodes->fake_mvoltage);
	if (ret != 0) {
		pr_uninode("Failed to read 'fakebatt-mvoltage'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_u32(devnode_fakebatt, "fakebatt-capacity", &uninodes->fake_capacity);
	if (ret != 0) {
		pr_uninode("Failed to read 'fakebatt-capacity'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_string(devnode_battage, "battage-psy", &uninodes->battage_psy);
	if (ret != 0) {
		pr_uninode("Failed to read 'battage-psy'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_string(devnode_thermal, "xo-therm", &xo_therm_name);
	if (ret != 0) {
		pr_uninode("Failed to read 'xo-therm'(%d)\n", ret);
		result = false;
	}

	ret = of_property_read_string(devnode_thermal, "bd-therm", &bd_therm_name);
	if (ret != 0) {
		pr_uninode("Failed to read 'bd-therm'(%d)\n", ret);
		result = false;
	}

	uninodes->xo_tz= thermal_zone_get_zone_by_name(xo_therm_name);
	if (IS_ERR(uninodes->xo_tz)) {
		pr_uninode("Failed to read 'xo_tz' (%d)\n", PTR_ERR(uninodes->xo_tz));
		uninodes->xo_tz = NULL;
		result = false;
	}

	uninodes->bd_tz= thermal_zone_get_zone_by_name(bd_therm_name);
	if (IS_ERR(uninodes->bd_tz)) {
		pr_uninode("Failed to read 'bd_tz' (%d)\n", PTR_ERR(uninodes->bd_tz));
		uninodes->bd_tz = NULL;
		result = false;
	}

	return result;
}

static ssize_t status_lcd_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*   ref;
	bool*           ori;
	int         new;

	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->status_lcd;

		sscanf(buf, "%d", &new);
		new = !!new;
		if (*ori != new)
			*ori = new;
	}

	return size;
}
static ssize_t status_lcd_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;

	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->status_lcd;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static int charging_restriction_limit(struct device_node* parent,
	const char* voter_name, const char* voter_type, const char* command) {
	int limit = VOTE_TOTALLY_RELEASED;

	struct device_node*	child;
	for_each_child_of_node(parent, child) {
		const char	*name, *type;
		u32		val;
		int		ret;

		ret = of_property_read_string(child, "restrict-name", &name);
		if (ret != 0) {
			pr_uninode("Failed to read 'restrict-name'(%d)\n", ret);
			break;
		}

		ret = strcmp(voter_name, name);
		if (ret != 0) {
			pr_debug("Continue to search %s\n", name);
			continue;
		}

		ret = of_property_read_string(child, "restrict-type", &type);
		if (ret != 0) {
			pr_uninode("Failed to read 'restrict-type'(%d)\n", ret);
			break;
		}

		ret = strcmp(voter_type, type);
		if (ret != 0) {
			pr_debug("Continue to search %s\n", type);
			continue;
		}

		ret = of_property_match_string(child, "restrict-commands", command);
		if (ret < 0) {
			pr_uninode("Failed to read 'restrict-commands' %s, (%d)\n", command, ret);
			break;
		}

		ret = of_property_read_u32_index(child, "restrict-values", ret, &val);
		if (ret != 0) {
			pr_uninode("Failed to read 'restrict-values'[%s], (%d)\n", command, ret);
			break;
		}

		if (val != 0) {
			pr_uninode("Finally success to find limit value %d for '%s(%s) %s'\n",
				val, voter_name, voter_type, command);
			limit = (int)val;
			break;
		}
	}

	return limit;
}
static ssize_t charging_restriction_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes* uninodes;
	struct voter_entry* restrictions;
	const char* space = strchr(buf, ' ');
	struct voter_entry* iter;
	char cmd[16] = { 0, };
	int i, limit;

	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		uninodes = (struct unified_nodes*)dev->platform_data;
		restrictions = uninodes->charging_restriction;

		strncpy(cmd, space+1, 15);
		strreplace(cmd, '\n', '\0');

		for (i = 0; i < RESTRICTION_MAX_COUNT; i++) {
			iter = &restrictions[i];

			if (!strncasecmp(buf, iter->name, space-buf)) {
				limit = charging_restriction_limit(uninodes->devnode_restrictchg,
					iter->name, vote_title(iter->type), cmd);

				if (!strcmp(iter->name, "LCD"))
					uninodes->status_lcd = !(limit == VOTE_TOTALLY_RELEASED);

				pr_uninode("%s votes %d to %s\n", iter->name, limit, vote_title(iter->type));
				veneer_voter_set(iter, limit);
			}
		}
	}
	else
		pr_uninode("Error on setting restriction\n");

	return size;
}
static ssize_t charging_restriction_show(struct device* dev, struct device_attribute* attr, char* buf) {
	// Do print logs, but Don't return anything
	int i;
	struct voter_entry* restrictions;
	struct voter_entry* restriction;

	if (dev && dev->platform_data) {
		restrictions = ((struct unified_nodes*)dev->platform_data)->charging_restriction;

		for (i = 0; i < RESTRICTION_MAX_COUNT; i++) {
			restriction = &restrictions[i];
			if (restriction->type != VOTER_TYPE_INVALID) {
				pr_dbg_uninode("voting Name=%s, Value=%d\n", restriction->name,
					(restriction->limit == VOTE_TOTALLY_RELEASED ? -1 : restriction->limit));
			}
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", -1);
}

static ssize_t status_boot_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct power_supply* wireless_psy;
	struct unified_nodes* ref;
	union power_supply_propval value = { .intval = 0, };
	int* ori;
	int new;

	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->status_boot;

		sscanf(buf, "%d", &new);
		new = !!new;
		if (*ori != new) {
			*ori = new;
			if (*ori) {
				wireless_psy = power_supply_get_by_name("wireless");
				if (wireless_psy) {
					power_supply_set_property(wireless_psy,
						POWER_SUPPLY_PROP_INPUT_SUSPEND, &value);
					power_supply_put(wireless_psy);
				}
			}
		}
	}

	return size;
}
static ssize_t status_boot_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;

	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->status_boot;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t thermald_iusb_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct voter_entry* iusb = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_iusb.type == VOTER_TYPE_IUSB)
		iusb = &((struct unified_nodes*)dev->platform_data)->thermald_iusb;
	else
		pr_uninode("Error on getting voter\n");

	return iusb ? voter_store(iusb, buf, size) : size;
}
static ssize_t thermald_iusb_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct voter_entry* iusb = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_iusb.type == VOTER_TYPE_IUSB)
		iusb = &((struct unified_nodes*)dev->platform_data)->thermald_iusb;
	else
		pr_uninode("Error on getting voter\n");

	return iusb ? voter_show(iusb, buf) : 0;
}

static ssize_t thermald_ibat_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct voter_entry* ibat = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_ibat.type == VOTER_TYPE_IBAT)
		ibat = &((struct unified_nodes*)dev->platform_data)->thermald_ibat;
	else
		pr_uninode("Error on getting voter\n");

	return ibat ? voter_store(ibat, buf, size) : size;
}
static ssize_t thermald_ibat_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct voter_entry* ibat = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_ibat.type == VOTER_TYPE_IBAT)
		ibat = &((struct unified_nodes*)dev->platform_data)->thermald_ibat;
	else
		pr_uninode("Error on getting voter\n");

	return ibat ? voter_show(ibat, buf) : 0;
}

static ssize_t thermald_idc_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct voter_entry* idc = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_idc.type == VOTER_TYPE_IDC)
		idc = &((struct unified_nodes*)dev->platform_data)->thermald_idc;
	else
		pr_uninode("Error on getting voter\n");

	return idc ? voter_store(idc, buf, size) : size;
}
static ssize_t thermald_idc_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct voter_entry* idc = NULL;

	if (dev && dev->platform_data &&
		((struct unified_nodes*)dev->platform_data)->thermald_idc.type == VOTER_TYPE_IDC)
		idc = &((struct unified_nodes*)dev->platform_data)->thermald_idc;
	else
		pr_uninode("Error on getting voter\n");

	return idc ? voter_show(idc, buf) : 0;
}

static ssize_t thermald_vdc_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct power_supply* wireless = power_supply_get_by_name("wireless");

	if (dev && dev->platform_data) {
		int value = -1;
		sscanf(buf, "%d", &value);

		if (wireless) {
			union power_supply_propval voltage = { .intval = value, };
			power_supply_set_property(wireless, POWER_SUPPLY_PROP_VOLTAGE_MAX, &voltage);
			power_supply_put(wireless);
		}
	} else
		pr_uninode("Error on getting voter\n");

	return size;
}
static ssize_t thermald_vdc_show(struct device* dev, struct device_attribute* attr, char* buf) {
	union power_supply_propval voltage = { .intval = 0, };
	static struct power_supply* wireless = NULL;
	int rc = 0;

	if (!wireless)
		wireless = power_supply_get_by_name("wireless");

	if (dev && dev->platform_data) {
		if (wireless) {
			rc = power_supply_get_property(wireless, POWER_SUPPLY_PROP_VOLTAGE_MAX, &voltage);
			if (!rc)
				power_supply_put(wireless);
		}
	} else
		pr_uninode("Error on getting voter\n");

	return snprintf(buf, PAGE_SIZE, "%d", voltage.intval);
}

static ssize_t charging_enable_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	struct voter_entry* iusb;
	struct voter_entry* idc;
	int enable = true;
	int limit;
	static int pre_enable = -1;

	sscanf(buf, "%d", &enable);
	if (pre_enable != enable) {
		pre_enable = enable;
		pr_uninode("Storing %s\n", buf);
	}

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		iusb = &ref->charging_enable[0];
		idc = &ref->charging_enable[1];

		if (iusb->type == VOTER_TYPE_IUSB && idc->type == VOTER_TYPE_IDC) {
			limit = !!enable ? VOTE_TOTALLY_RELEASED : VOTE_TOTALLY_BLOCKED;

			veneer_voter_set(iusb, limit);
			veneer_voter_set(idc, limit);

			pr_uninode("Success to set charging_enabled\n");
		}
	}

	return size;
}
static ssize_t charging_enable_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 1;
	struct unified_nodes*	ref;
	struct voter_entry* iusb;
	struct voter_entry* idc;

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		iusb = &ref->charging_enable[0];
		idc = &ref->charging_enable[1];

		if (iusb->type == VOTER_TYPE_IUSB && idc->type == VOTER_TYPE_IDC
			&& iusb->limit == idc->limit) {
			ret = (iusb->limit == VOTE_TOTALLY_RELEASED);
			pr_dbg_uninode("Success to get charging_enabled (%d)\n", ret);
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

enum charging_step_index {
	DISCHG = 0,
	TRICKLE,
	PRECHARGING,
	FAST,
	FULLON,
	TAPER,
	EOC,
	INHIBIT,
};

static const char* const charging_step_string [] = {
	"0 DISCHG",
	"1 TRICKLE",
	"2 PRECHARGING",
	"3 FAST",
	"4 FULLON",
	"5 TAPER",
	"6 EOC",
	"7 INHIBIT",
};

static ssize_t charging_step_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	const char* ret = charging_step_string[DISCHG];
	struct unified_nodes*	ref;
	int 		new;

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		sscanf(buf, "%d", &new);

		switch (new) {
		case CHARGING_STEP_TRICKLE:	ret = charging_step_string[TRICKLE];
			break;
		case CHARGING_STEP_CC:		ret = charging_step_string[FAST];
			break;
		case CHARGING_STEP_CV:		ret = charging_step_string[TAPER];
			break;
		case CHARGING_STEP_TERMINATED:	ret = charging_step_string[EOC];
			break;
		default:
			break;
		}

		ref->charging_step = ret;
	}

	return size;
}
static ssize_t charging_step_show(struct device* dev, struct device_attribute* attr, char* buf) {
	const char* ret = "0 DISCHG";
	struct unified_nodes*	ref;

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		if (ref->charging_step)
			ret = ref->charging_step;
	}

	return snprintf(buf, PAGE_SIZE, "%s", ret);
}

static ssize_t charging_showcase_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	bool*			ori;
	int 		new;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->charging_showcase;

		sscanf(buf, "%d", &new);
		new = !!new;
		if (*ori != new) {
			*ori = new;
			protection_showcase_update();
		}
	}

	return size;
}
static ssize_t charging_showcase_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;

	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->charging_showcase;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t charging_completed_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 1;
	struct power_supply* battery = power_supply_get_by_name("battery");
	union power_supply_propval capacity = { .intval = 0, };

	if (dev && dev->platform_data) {
		if (battery) {
			if (!power_supply_get_property(battery, POWER_SUPPLY_PROP_CAPACITY, &capacity)
				&& capacity.intval == 100)
				ret = 0;
			power_supply_put(battery);
		}
	}
	pr_uninode("returning chcomp to %d", ret);
	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t fake_battery_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	bool*			ori;
	int			new;
	struct power_supply* psy;
	union power_supply_propval prp_temp, prp_capacity, prp_voltagenow;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->fake_battery;

		sscanf(buf, "%d", &new);
		new = !!new;
		if (*ori != new) {
			psy = power_supply_get_by_name(ref->fake_psy);

			prp_temp.intval = new ? ref->fake_temperature : -1;
			prp_capacity.intval = new ? ref->fake_capacity : -1;
			prp_voltagenow.intval = new ? ref->fake_mvoltage*1000 : -1;
			if (psy) {
				if (!power_supply_set_property(psy, POWER_SUPPLY_PROP_TEMP, &prp_temp)
					&& !power_supply_set_property(psy, POWER_SUPPLY_PROP_CAPACITY, &prp_capacity)
					&& !power_supply_set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prp_voltagenow))
					*ori = new;
				power_supply_put(psy);
			}
		}
	}

	return size;
}
static ssize_t fake_battery_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->fake_battery;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t fake_sdpmax_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	bool*			ori;
	int 		new;

	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->fake_sdpmax;

		sscanf(buf, "%d", &new);
		new = !!new;
		if (*ori != new) {
			*ori = new;
			charging_ceiling_sdpmax(new);
		}
	}

	return size;
}
static ssize_t fake_sdpmax_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->fake_sdpmax;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t fake_hvdcp_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct power_supply*	psy;
	struct unified_nodes*	ref;
	bool*			ori;
	int 		new;
	union power_supply_propval cmd;

	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		psy = power_supply_get_by_name("usb");
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->fake_hvdcp;

		sscanf(buf, "%d", &new);
		new = !!new;

		if (*ori != new) {
			if (psy) {
				*ori = new;
				cmd.intval = new;
				power_supply_set_property(psy, POWER_SUPPLY_PROP_USB_HC, &cmd);
				power_supply_put(psy);
			}
		}
	}

	return size;
}
static ssize_t fake_hvdcp_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->fake_hvdcp;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t battery_age_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	struct unified_nodes*	ref;
	struct power_supply* capacity_psy = NULL;
	union power_supply_propval capacity_designed = { .intval = 0, }, capacity_learned = { .intval = 0, };
	static int pre_designed, pre_learned = 0;

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		if (ref->battage_psy)
			capacity_psy = power_supply_get_by_name(ref->battage_psy);

		if (capacity_psy
			&& !power_supply_get_property(capacity_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &capacity_designed)
			&& !power_supply_get_property(capacity_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL, &capacity_learned)
			&& capacity_designed.intval > 0) {

			ret = min((capacity_learned.intval * 100) / capacity_designed.intval,
				100);

			if (pre_designed != capacity_learned.intval ||pre_learned != capacity_designed.intval) {
				pre_designed = capacity_learned.intval;
				pre_learned = capacity_designed.intval;
				pr_uninode("battery age %d = %d/%d\n",
					ret, capacity_learned.intval, capacity_designed.intval);
			}
		}
		else
			pr_uninode("Calculating battery age is not ready\n");

		if (capacity_psy)
			power_supply_put(capacity_psy);
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}
static ssize_t battery_condition_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int age;	// ratio type for battery age, i.e 95(%)
	int ret = 0;
	int age_thresold_best = 80;
	int age_thresold_good = 50;
	int age_thresold_bad = 0;
	int condition_ret_best = 1;
	int condition_ret_good = 2;
	int condition_ret_bad = 3;

	if (!!battery_age_show(dev, attr, buf) && !!sscanf(buf, "%d", &age) && age >= 0) {
		if (age >= age_thresold_best)
			ret = condition_ret_best;
		else if (age >= age_thresold_good)
			ret = condition_ret_good;
		else if (age >= age_thresold_bad)
			ret = condition_ret_bad;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t battery_cycle_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	struct unified_nodes* ref;
	struct power_supply* battery_psy = NULL;
	union power_supply_propval cycle_count = { .intval = 0, };

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		if (ref->battage_psy)
			battery_psy = power_supply_get_by_name(ref->battage_psy);

		if (battery_psy
			&& !power_supply_get_property(battery_psy,
				POWER_SUPPLY_PROP_CYCLE_COUNT, &cycle_count)) {
			ret = cycle_count.intval;
		}
		if (battery_psy)
			power_supply_put(battery_psy);
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t battery_valid_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	bool*	ori;
	int new;
	pr_uninode("Storing %s\n", buf);

	if (dev && dev->platform_data) {
		ori = &((struct unified_nodes*)dev->platform_data)->battery_valid;
		sscanf(buf, "%d", &new);
		*ori = !!new;
	}

	return size;
}
static ssize_t battery_valid_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;

	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->battery_valid;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t charger_name_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	char* ori;

	if (dev && dev->platform_data) {
		ori = ((struct unified_nodes*)dev->platform_data)->charger_name;
		if (!strcmp(ori, buf))
			pr_uninode("Storing %s\n", buf);

		strcpy(ori, buf);
		ori[strlen(buf)] = '\0';
	}

	return size;
}
static ssize_t charger_name_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	char* ori;

	if (dev && dev->platform_data) {
		ori = ((struct unified_nodes*)dev->platform_data)->charger_name;
		strncpy(buf, ori, strlen(ori));
		ret = strlen(ori);
	}

	return ret;
}

static ssize_t charger_highspeed_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	bool*	ori;
	int new;
	static int pre_new = -1;

	sscanf(buf, "%d", &new);
	if (new != pre_new) {
		pre_new = new;
		pr_uninode("Storing %s\n", buf);
	}

	if (dev && dev->platform_data) {
		ori = &((struct unified_nodes*)dev->platform_data)->charger_highspeed;

		*ori = !!new;
	}

	return size;
}
static ssize_t charger_highspeed_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->charger_highspeed;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t charger_incompatible_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	static bool container = false;
	bool**	ori;
	int new = -1;
	static int pre_new = -1;

	if (dev && dev->platform_data) {
		ori = &((struct unified_nodes*)dev->platform_data)->charger_incompatible;
		if (!(*ori))
			*ori = &container;

		sscanf(buf, "%d", &new);
		container = !!new;
	}

	if (new != pre_new) {
		pre_new = new;
		pr_uninode("Storing %s\n", buf);
	}
	return size;
}
static ssize_t charger_incompatible_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = UNIFIED_NODES_DISABLED;
	bool* val;

	if (dev && dev->platform_data) {
		val = ((struct unified_nodes*)dev->platform_data)->charger_incompatible;
		if (val)
			ret = !!(*val);
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t charger_usbid_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct power_supply* 		usb = power_supply_get_by_name("usb");
	union power_supply_propval	prp = { .intval = 0, };
	int rc = 0;

	if (dev && dev->platform_data) {
		if (usb) {
			rc = power_supply_get_property(usb, POWER_SUPPLY_PROP_RESISTANCE, &prp);
			if (!rc)
				power_supply_put(usb);
		}
	}

	pr_uninode("returning usbid(mv) is %d\n", prp.intval);
	return snprintf(buf, PAGE_SIZE, "Result:%d Raw:ffff\n", prp.intval);
}

static ssize_t charger_verbose_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	static enum charging_verbosity container = VERBOSE_CHARGER_NONE;
	enum charging_verbosity** ori;
	int 		  new;

	if (dev && dev->platform_data) {
		ori = &((struct unified_nodes*)dev->platform_data)->charger_verbose;

		if (!(*ori))
			*ori = &container;

		sscanf(buf, "%d", &new);
		container = new;
	}

	pr_uninode("Storing %s\n", buf);
	return size;
}
static ssize_t charger_verbose_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = UNIFIED_NODES_DISABLED;
	enum charging_verbosity* val;

	if (dev && dev->platform_data) {
		val = ((struct unified_nodes*)dev->platform_data)->charger_verbose;
		if (val)
			ret = *val;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t support_fastpl_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	int*			ori;
	int 		new = 0;

	pr_uninode("FASTPL: Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->support_fastpl;

		sscanf(buf, "%d", &new);
		// If parallel charging is not supported (*ori == -1),
		// then do not update it via this command.
		if (*ori >= 0 && new >= 0 && *ori != new) {
			*ori = new;
		}
	}
	return size;
}
static ssize_t support_fastpl_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->support_fastpl;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t support_fastchg_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;
	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->support_fastchg;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t bsm_timetofull_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t size) {
	struct unified_nodes*	ref;
	int*			ori;
	int 		new;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->bsm_timetofull;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			*ori = new;
			charging_time_update(CHARGING_SUPPLY_TYPE_NONE, true);
		}
	}

	return size;
}
static ssize_t bsm_timetofull_show(struct device* dev, struct device_attribute* attr, char* buf) {
	int ret = 0;

	if (dev && dev->platform_data) {
		ret = ((struct unified_nodes*)dev->platform_data)->bsm_timetofull;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t factory_xo_therm_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct thermal_zone_device *tz;
	int sens_temp = -1;
	int ret = 0;

	if (dev && dev->platform_data) {
		tz = ((struct unified_nodes*)dev->platform_data)->xo_tz;
		ret = thermal_zone_get_temp(tz, &sens_temp);
		if (ret) {
			pr_uninode("xo therm read error:%d\n", ret);
		}
	}

	return snprintf(buf, PAGE_SIZE, "Result:%d Raw:ffff\n", sens_temp);
}

static ssize_t factory_bd_therm_show(struct device* dev, struct device_attribute* attr, char* buf) {
	struct thermal_zone_device *tz;
	int sens_temp = -1;
	int ret = 0;

	if (dev && dev->platform_data) {
		tz = ((struct unified_nodes*)dev->platform_data)->bd_tz;
		ret = thermal_zone_get_temp(tz, &sens_temp);
		if (ret) {
			pr_uninode("bd therm read error:%d\n", ret);
		}
	}

	return snprintf(buf, PAGE_SIZE, "Result:%d Raw:ffff\n", sens_temp);
}

#define MAX_UNIFIED_NODES_SIZE	30
static struct device_attribute unified_nodes_dattrs[];
static int find_actm_attr_id(struct device_attribute* attr)
{
	int i = 0;

	for (i = 0; i < MAX_UNIFIED_NODES_SIZE; i++)
		if (attr == &unified_nodes_dattrs[i])
			break;

	if (i == MAX_UNIFIED_NODES_SIZE)
		return -1;

	return i;
}

static ssize_t actm_mode_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->actm.mode;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			if (new == -1 ||
				new ==  0 || new ==  1 || new ==  2 || new ==  3 ||
				new == 10 || new == 11 || new == 12 || new == 13 ||
				new == 20 || new == 21 || new == 22 || new == 23 ||
				new == 30 || new == 31 || new == 32 || new == 33 )
				*ori = new;
		}
	}

	return size;
}

static ssize_t actm_mode_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	int ret = -2;

	if (dev && dev->platform_data)
		ret = ((struct unified_nodes*)dev->platform_data)->actm.mode;

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t actm_lcdon_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->actm.lcdon_temp_offset;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			*ori = new;
		}
	}

	return size;
}

static ssize_t actm_lcdon_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	int ret = -2;
	struct unified_nodes* ref = NULL;

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		if (ref->status_lcd)
			ret = ref->actm.lcdon_temp_offset;
		else
			ret = 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}


static ssize_t actm_sensor_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;
	int id = find_actm_attr_id(attr);

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;

		switch (id) {
			case 2: ori = &ref->actm.sensor_type[0]; break;
			case 3: ori = &ref->actm.sensor_type[1]; break;
		}

		if (ori) {
			sscanf(buf, "%d", &new);
			if (*ori != new) {
				if (new == 1 || new == 2 || new == 3)
					*ori = new;
			}
		}
	}

	return size;
}

static ssize_t actm_sensor_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	struct unified_nodes *ref = NULL;
	int ret = -2, id = find_actm_attr_id(attr);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		switch (id) {
			case 2: ret = ref->actm.sensor_type[0]; break;
			case 3: ret = ref->actm.sensor_type[1]; break;
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t actm_holddeg_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;
	int id = find_actm_attr_id(attr);

	pr_uninode("id = %d, Storing %s\n", id, buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;

		switch (id) {
			case 4: ori = &ref->actm.max_hold_criteria_wired[0]; break;
			case 5: ori = &ref->actm.max_hold_criteria_wired[1]; break;
			case 6: ori = &ref->actm.max_hold_criteria_wired[2]; break;
			case 7: ori = &ref->actm.max_hold_criteria_wireless[0]; break;
			case 8: ori = &ref->actm.max_hold_criteria_wireless[1]; break;
			case 9: ori = &ref->actm.max_hold_criteria_wireless[2]; break;
		}

		if (ori) {
			sscanf(buf, "%d", &new);
			if (*ori != new && new > 0)
				*ori = new;
		}
	}

	return size;
}

static ssize_t actm_holddeg_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	struct unified_nodes *ref = NULL;
	int ret = -9999, id = find_actm_attr_id(attr);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		switch (id) {
			case 4: ret = ref->actm.max_hold_criteria_wired[0]; break;
			case 5: ret = ref->actm.max_hold_criteria_wired[1]; break;
			case 6: ret = ref->actm.max_hold_criteria_wired[2]; break;
			case 7: ret = ref->actm.max_hold_criteria_wireless[0]; break;
			case 8: ret = ref->actm.max_hold_criteria_wireless[1]; break;
			case 9: ret = ref->actm.max_hold_criteria_wireless[2]; break;
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t actm_tempoffs_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;
	int id = find_actm_attr_id(attr);

	pr_uninode("id = %d, Storing %s\n", id, buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;

		switch (id) {
			case 10: ori = &ref->actm.tempoffs_wired[0]; break;
			case 11: ori = &ref->actm.tempoffs_wired[1]; break;
			case 12: ori = &ref->actm.tempoffs_wired[2]; break;
			case 13: ori = &ref->actm.tempoffs_wireless[0]; break;
			case 14: ori = &ref->actm.tempoffs_wireless[1]; break;
			case 15: ori = &ref->actm.tempoffs_wireless[2]; break;
		}

		if (ori) {
			sscanf(buf, "%d", &new);
			if (*ori != new)
				*ori = new;
		}
	}

	return size;
}

static ssize_t actm_tempoffs_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	struct unified_nodes *ref = NULL;
	int ret = -9999, id = find_actm_attr_id(attr);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		switch (id) {
			case 10: ret = ref->actm.tempoffs_wired[0]; break;
			case 11: ret = ref->actm.tempoffs_wired[1]; break;
			case 12: ret = ref->actm.tempoffs_wired[2]; break;
			case 13: ret = ref->actm.tempoffs_wireless[0]; break;
			case 14: ret = ref->actm.tempoffs_wireless[1]; break;
			case 15: ret = ref->actm.tempoffs_wireless[2]; break;
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t actm_current_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;
	int id = find_actm_attr_id(attr);

	pr_uninode("id = %d, Storing %s\n", id, buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;

		switch (id) {
			case 16: ori = &ref->actm.current_wired[0]; break;
			case 17: ori = &ref->actm.current_wired[1]; break;
			case 18: ori = &ref->actm.current_wired[2]; break;
			case 19: ori = &ref->actm.power_epp[0]; break;
			case 20: ori = &ref->actm.power_epp[1]; break;
			case 21: ori = &ref->actm.power_epp[2]; break;
			case 22: ori = &ref->actm.power_bpp[0]; break;
			case 23: ori = &ref->actm.power_bpp[1]; break;
			case 24: ori = &ref->actm.power_bpp[2]; break;
			case 25: ori = &ref->actm.current_cp[0]; break;
			case 26: ori = &ref->actm.current_cp[1]; break;
			case 27: ori = &ref->actm.wired_max_fcc[0]; break;
			case 28: ori = &ref->actm.wired_max_fcc[1]; break;
			case 29: ori = &ref->actm.wired_max_fcc[2]; break;
		}

		if (ori) {
			sscanf(buf, "%d", &new);
			if (*ori != new && new > 0)
				*ori = new;
		}
	}

	return size;
}

static ssize_t actm_current_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	struct unified_nodes *ref = NULL;
	int ret = -9999, id = find_actm_attr_id(attr);

	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		switch (id) {
			case 16: ret = ref->actm.current_wired[0]; break;
			case 17: ret = ref->actm.current_wired[1]; break;
			case 18: ret = ref->actm.current_wired[2]; break;
			case 19: ret = ref->actm.power_epp[0]; break;
			case 20: ret = ref->actm.power_epp[1]; break;
			case 21: ret = ref->actm.power_epp[2]; break;
			case 22: ret = ref->actm.power_bpp[0]; break;
			case 23: ret = ref->actm.power_bpp[1]; break;
			case 24: ret = ref->actm.power_bpp[2]; break;
			case 25: ret = ref->actm.current_cp[0]; break;
			case 26: ret = ref->actm.current_cp[1]; break;
			case 27: ret = ref->actm.wired_max_fcc[0]; break;
			case 28: ret = ref->actm.wired_max_fcc[1]; break;
			case 29: ret = ref->actm.wired_max_fcc[2]; break;
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t irc_enabled_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->irc_enabled;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			*ori = !!new;
		}
	}

	return size;
}

static ssize_t irc_enabled_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	int ret = -2;

	if (dev && dev->platform_data)
		ret = ((struct unified_nodes*)dev->platform_data)->irc_enabled;

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t irc_resistance_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->irc_resistance;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			*ori = new;
		}
	}

	return size;
}

static ssize_t irc_resistance_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	int ret = -2;

	if (dev && dev->platform_data)
		ret = ((struct unified_nodes*)dev->platform_data)->irc_resistance;

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t pps_ta_count_store(
	struct device* dev, struct device_attribute* attr, const char* buf, size_t size)
{
	struct unified_nodes *ref = NULL;
	int *ori = NULL;
	int new = -9999;

	pr_uninode("Storing %s\n", buf);
	if (dev && dev->platform_data) {
		ref = (struct unified_nodes*)dev->platform_data;
		ori = &ref->pps_ta_count;

		sscanf(buf, "%d", &new);
		if (*ori != new) {
			*ori = new;
		}
	}

	return size;
}

static ssize_t pps_ta_count_show(
	struct device* dev, struct device_attribute* attr, char* buf)
{
	int ret = -2;

	if (dev && dev->platform_data)
		ret = ((struct unified_nodes*)dev->platform_data)->pps_ta_count;

	return snprintf(buf, PAGE_SIZE, "%d", ret);
}

static struct device_attribute unified_nodes_dattrs [] = {
/*  0 */__ATTR(actm_mode,                0664, actm_mode_show,     actm_mode_store    ),
/*  1 */__ATTR(actm_lcdon_offset,        0664, actm_lcdon_show,    actm_lcdon_store   ),
/*  2 */__ATTR(actm_sensor_wired,        0664, actm_sensor_show,   actm_sensor_store  ),
/*  3 */__ATTR(actm_sensor_wireless,     0664, actm_sensor_show,   actm_sensor_store  ),

/*  4 */__ATTR(actm_holddeg_wired_0,     0664, actm_holddeg_show,  actm_holddeg_store ),
/*  5 */__ATTR(actm_holddeg_wired_1,     0664, actm_holddeg_show,  actm_holddeg_store ),
/*  6 */__ATTR(actm_holddeg_wired_2,     0664, actm_holddeg_show,  actm_holddeg_store ),
/*  7 */__ATTR(actm_holddeg_wireless_0,  0664, actm_holddeg_show,  actm_holddeg_store ),
/*  8 */__ATTR(actm_holddeg_wireless_1,  0664, actm_holddeg_show,  actm_holddeg_store ),
/*  9 */__ATTR(actm_holddeg_wireless_2,  0664, actm_holddeg_show,  actm_holddeg_store ),

/* 10 */__ATTR(actm_tempoffs_wired_0,    0664, actm_tempoffs_show, actm_tempoffs_store),
/* 11 */__ATTR(actm_tempoffs_wired_1,    0664, actm_tempoffs_show, actm_tempoffs_store),
/* 12 */__ATTR(actm_tempoffs_wired_2,    0664, actm_tempoffs_show, actm_tempoffs_store),
/* 13 */__ATTR(actm_tempoffs_wireless_0, 0664, actm_tempoffs_show, actm_tempoffs_store),
/* 14 */__ATTR(actm_tempoffs_wireless_1, 0664, actm_tempoffs_show, actm_tempoffs_store),
/* 15 */__ATTR(actm_tempoffs_wireless_2, 0664, actm_tempoffs_show, actm_tempoffs_store),

/* 16 */__ATTR(actm_current_wired_0,     0664, actm_current_show,  actm_current_store ),
/* 17 */__ATTR(actm_current_wired_1,     0664, actm_current_show,  actm_current_store ),
/* 18 */__ATTR(actm_current_wired_2,     0664, actm_current_show,  actm_current_store ),
/* 19 */__ATTR(actm_power_epp_0,         0664, actm_current_show,  actm_current_store ),
/* 20 */__ATTR(actm_power_epp_1,         0664, actm_current_show,  actm_current_store ),
/* 21 */__ATTR(actm_power_epp_2,         0664, actm_current_show,  actm_current_store ),
/* 22 */__ATTR(actm_power_bpp_0,         0664, actm_current_show,  actm_current_store ),
/* 23 */__ATTR(actm_power_bpp_1,         0664, actm_current_show,  actm_current_store ),
/* 24 */__ATTR(actm_power_bpp_2,         0664, actm_current_show,  actm_current_store ),
/* 25 */__ATTR(actm_curr_cp_pps,         0664, actm_current_show,  actm_current_store ),
/* 26 */__ATTR(actm_curr_cp_qc30,        0664, actm_current_show,  actm_current_store ),

/* 27 */__ATTR(actm_max_fcc_pps,         0664, actm_current_show,  actm_current_store ),
/* 28 */__ATTR(actm_max_fcc_qc3,         0664, actm_current_show,  actm_current_store ),
/* 29 */__ATTR(actm_max_fcc_qc2,         0664, actm_current_show,  actm_current_store ),

/* 30 ==> must check "#define MAX_UNIFIED_NODES_SIZE 30" */

    __ATTR(irc_enabled,         0664, irc_enabled_show,    irc_enabled_store   ),
    __ATTR(irc_resistance,      0664, irc_resistance_show, irc_resistance_store),

	__ATTR(thermald_iusb,		0664, thermald_iusb_show,		thermald_iusb_store),
	__ATTR(thermald_ibat,		0664, thermald_ibat_show,		thermald_ibat_store),
	__ATTR(thermald_idc,		0664, thermald_idc_show,		thermald_idc_store),
	__ATTR(thermald_vdc,		0664, thermald_vdc_show,		thermald_vdc_store),
	__ATTR(status_boot,		0664, status_boot_show,			status_boot_store),
	__ATTR(status_lcd,		0664, status_lcd_show,			status_lcd_store),
	__ATTR(charging_restriction,	0664, charging_restriction_show,	charging_restriction_store),
	__ATTR(charging_enable,		0664, charging_enable_show,		charging_enable_store),
	__ATTR(charging_step,		0644, charging_step_show,		charging_step_store),
	__ATTR(charging_showcase,	0664, charging_showcase_show,		charging_showcase_store),
	__ATTR(charging_completed,	0444, charging_completed_show,		NULL),
	__ATTR(fake_battery,		0664, fake_battery_show,		fake_battery_store),
	__ATTR(fake_sdpmax,		0664, fake_sdpmax_show,			fake_sdpmax_store),
	__ATTR(fake_hvdcp,		0664, fake_hvdcp_show, 			fake_hvdcp_store),
	__ATTR(battery_age,		0444, battery_age_show,			NULL),
	__ATTR(battery_condition,	0444, battery_condition_show,		NULL),
	__ATTR(battery_cycle,		0444, battery_cycle_show,		NULL),
	__ATTR(battery_valid,		0664, battery_valid_show,		battery_valid_store),
	__ATTR(charger_name,		0664, charger_name_show,		charger_name_store),
	__ATTR(charger_highspeed,	0664, charger_highspeed_show,		charger_highspeed_store),
	__ATTR(charger_incompatible,	0664, charger_incompatible_show,	charger_incompatible_store),
	__ATTR(charger_usbid,		0444, charger_usbid_show,		NULL),
	__ATTR(charger_verbose,		0664, charger_verbose_show,		charger_verbose_store),
	__ATTR(support_fastpl,		0664, support_fastpl_show,		support_fastpl_store),
	__ATTR(support_fastchg,		0444, support_fastchg_show,		NULL),
	__ATTR(bsm_timetofull,		0664, bsm_timetofull_show,	bsm_timetofull_store),
	__ATTR(factory_xo_therm, 	0444, factory_xo_therm_show, 	NULL),
	__ATTR(factory_bd_therm, 	0444, factory_bd_therm_show, 	NULL),
	__ATTR(pps_ta_count,		0444, pps_ta_count_show, pps_ta_count_store),

};

static struct platform_device unified_nodes_device = {
	.name = UNIFIED_NODES_DEVICE,
	.id = -1,	// Set -1 explicitly to make device name simple
	.dev = {
		.platform_data = NULL,
	}
};

static void unified_nodes_clear(/*@Nonnull*/ struct unified_nodes* uninodes) {
	memset(uninodes, 0, sizeof(struct unified_nodes));
	// It makes all the voter_entry.type to INVALID(0)
}

static struct device_attribute* unified_nodes_search(const char* key) {
       /*	struct device_attribute {
	*		struct attribute attr;
	*		ssize_t (*show)(struct device* dev, struct device_attribute* attr, char* buf);
	*		ssize_t (*store)(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);
	*	};
	*	struct attribute {
	*		const char* name;
	*		struct module* owner;
	*		mode_t mode;
	*	};
	*	#define __ATTR(_name,_mode,_show,_store) { .attr = {.name = __stringify(_name), .mode = _mode, .owner = THIS_MODULE }, .show = _show, .store = _store, }
	*/
	int i;
	for (i = 0; i < ARRAY_SIZE(unified_nodes_dattrs); i++) {
		if (!strcmp(unified_nodes_dattrs[i].attr.name, key)) {
			return &unified_nodes_dattrs[i];
		}
	}

	return NULL;
}

bool unified_nodes_store(const char* key, const char* value, size_t size) {
	struct device* device = &unified_nodes_device.dev;
	struct device_attribute* dattr = unified_nodes_search(key);
	if (dattr) {
		dattr->store(device, dattr, value, size);
		return true;
	}
	else
		return false;
}

bool unified_nodes_show(const char* key, char* value) {
	struct device* device = &unified_nodes_device.dev;
	struct device_attribute* dattr = unified_nodes_search(key);
	if (dattr) {
		dattr->show(device, dattr, value);
		return true;
	}
	else
		return false;
}

static struct attribute_group* unified_nodes_attrgroup(void) {
	static struct attribute*	s_attrs [ARRAY_SIZE(unified_nodes_dattrs) + 1];
	static struct attribute_group	s_files = { .attrs  = NULL, };
	int i;

	if (s_files.attrs == NULL) {
		s_files.attrs = s_attrs;

		for (i = 0; i < ARRAY_SIZE(unified_nodes_dattrs); i++)
			s_attrs[i] = &unified_nodes_dattrs[i].attr;
		s_attrs[i] = NULL;
	}

	return &s_files;
}

bool unified_nodes_create(struct device_node* devnode)
{
	struct unified_nodes* uninodes =
		kzalloc(sizeof(struct unified_nodes), GFP_KERNEL);

	pr_uninode("Creating unified nodes\n");

	if (!uninodes) {
		pr_uninode("Failed to alloc unified_nodes\n");
		goto failed;
	}
	else
		unified_nodes_device.dev.platform_data = uninodes;

	if (!unified_nodes_devicetree(devnode, uninodes)) {
		pr_uninode("Failed to parse unified_nodes_devicetree\n");
		goto failed;
	}

	if (!unified_nodes_actm_dt(devnode, uninodes)) {
		pr_uninode("Failed to parse unified_nodes_actm_dt\n");
		goto failed;
	}

	if (!voter_register(uninodes)) {
		pr_uninode("Failed to preset voters\n");
		goto failed;
	}
	if (sysfs_create_group(
			&unified_nodes_device.dev.kobj, unified_nodes_attrgroup())) {
		pr_uninode("Failed to create sysfs voters\n");
		goto failed;
	}

	pr_uninode("Success to create unified nodes\n");
	return true;

failed:	unified_nodes_destroy();
	return false;
}

void unified_nodes_destroy(void) {
	struct unified_nodes* uninodes;

	pr_uninode("Destroying unified nodes\n");

	if (unified_nodes_device.dev.platform_data) {
		uninodes = unified_nodes_device.dev.platform_data;

		voter_unregister(uninodes);
		unified_nodes_clear(uninodes);
		kfree(uninodes);

		unified_nodes_device.dev.platform_data = NULL;
	}

	sysfs_remove_group(&unified_nodes_device.dev.kobj, unified_nodes_attrgroup());
}

static int __init unified_nodes_init(void) {
	pr_uninode("platform_device_register : unified_nodes_device\n");
	return platform_device_register(&unified_nodes_device);
}

static void __exit unified_nodes_exit(void) {
	pr_uninode("platform_device_unregister : unified_nodes_device\n");
	platform_device_unregister(&unified_nodes_device);
}

module_init(unified_nodes_init);
module_exit(unified_nodes_exit);
