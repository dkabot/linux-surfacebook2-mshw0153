#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>


#define SB2_SHPS_DSM_REVISION	1
#define SB2_SHPS_DSM_GPU_STATE	0x05

static const guid_t SB2_SHPS_DSM_UUID =
	GUID_INIT(0x5515a847, 0xed55, 0x4b27, 0x83, 0x52, 0xcd,
	          0x32, 0x0e, 0x10, 0x36, 0x0a);

#define SB2_PARAM_PERM	(S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)


static const struct acpi_gpio_params gpio_base_presence_int = { 0, 0, false };
static const struct acpi_gpio_params gpio_base_presence     = { 1, 0, false };
static const struct acpi_gpio_params gpio_dgpu_power_int    = { 2, 0, false };
static const struct acpi_gpio_params gpio_dgpu_power        = { 3, 0, false };
static const struct acpi_gpio_params gpio_dgpu_presence_int = { 4, 0, false };
static const struct acpi_gpio_params gpio_dgpu_presence     = { 5, 0, false };

static const struct acpi_gpio_mapping sb2_mshw0153_acpi_gpios[] = {
	{ "base_presence-int-gpio", &gpio_base_presence_int, 1 },
	{ "base_presence-gpio",     &gpio_base_presence,     1 },
	{ "dgpu_power-int-gpio",    &gpio_dgpu_power_int,    1 },
	{ "dgpu_power-gpio",        &gpio_dgpu_power,        1 },
	{ "dgpu_presence-int-gpio", &gpio_dgpu_presence_int, 1 },
	{ "dgpu_presence-gpio",     &gpio_dgpu_presence,     1 },
	{ },
};


static bool sb2_dgpu_default_pwr = false;

module_param_named(dgpu_pwr, sb2_dgpu_default_pwr, bool, SB2_PARAM_PERM);
MODULE_PARM_DESC(dgpu_pwr, "dGPU power state (on/off)");

extern struct proc_dir_entry *acpi_root_dir;
static acpi_handle shps;
static char state[4]; // I don't know how to actually check the status...
// TODO: sysfs interface


static int sb2_shps_dgpu_set_power(bool on)
{
	union acpi_object *result;
	union acpi_object param;

	param.type = ACPI_TYPE_INTEGER;
	param.integer.value = on;

	result = acpi_evaluate_dsm_typed(shps, &SB2_SHPS_DSM_UUID, SB2_SHPS_DSM_REVISION,
	                                 SB2_SHPS_DSM_GPU_STATE, &param, ACPI_TYPE_BUFFER);

	if (IS_ERR_OR_NULL(result)) {
		return result ? PTR_ERR(result) : -EFAULT;
	}

	if (result->buffer.length != 1 || result->buffer.pointer[0] != 0) {
		return -EIO;
	}

	printk(KERN_INFO "SB2 SHPS: dGPU power state set to \'%d\'\n", on);
    on ? strcpy(state, "ON") : strcpy(state, "OFF");

	ACPI_FREE(result);
	return 0;
}

static int sb2_shps_resume(struct device *dev)
{
	acpi_handle shps_handle = ACPI_HANDLE(dev);
    if (shps_handle != shps) {
        shps = shps_handle;
    }

	return sb2_shps_dgpu_set_power(sb2_dgpu_default_pwr);
}

static ssize_t bbswitch_proc_write(struct file *fp, const char __user *buff,
    size_t len, loff_t *off) {
    char cmd[8];

    if (len >= sizeof(cmd))
        len = sizeof(cmd) - 1;

    if (copy_from_user(cmd, buff, len))
        return -EFAULT;

    //dis_dev_get();

    if (strncmp(cmd, "OFF", 3) == 0)
        sb2_shps_dgpu_set_power(false); //bbswitch_off();

    if (strncmp(cmd, "ON", 2) == 0)
        sb2_shps_dgpu_set_power(true); //bbswitch_on();

    //dis_dev_put();

    return len;
}

static int bbswitch_proc_show(struct seq_file *seqfp, void *p) {
    // show the card state. Example output: 0000:01:00:00 ON
    /*dis_dev_get();
    seq_printf(seqfp, "%s %s\n", dev_name(&dis_dev->dev),
             is_card_disabled() ? "OFF" : "ON");
    dis_dev_put();*/
    
    seq_printf(seqfp, "0000:02:00.0 %s\n", state);
    return 0;
}
static int bbswitch_proc_open(struct inode *inode, struct file *file) {
    return single_open(file, bbswitch_proc_show, NULL);
}

static struct file_operations bbswitch_fops = {
    .open   = bbswitch_proc_open,
    .read   = seq_read,
    .write  = bbswitch_proc_write,
    .llseek = seq_lseek,
    .release= single_release
};

static int sb2_shps_probe(struct platform_device *pdev)
{
    struct proc_dir_entry *acpi_entry;
	struct acpi_device *shps_dev = ACPI_COMPANION(&pdev->dev);
	int status = 0;
    
    shps = ACPI_HANDLE(&pdev->dev);

	if (gpiod_count(&pdev->dev, NULL) < 0) {
		return -ENODEV;
	}

	status = acpi_dev_add_driver_gpios(shps_dev, sb2_mshw0153_acpi_gpios);
	if (status) {
		return status;
	}

	status = sb2_shps_dgpu_set_power(sb2_dgpu_default_pwr);
	if (status) {
		acpi_dev_remove_driver_gpios(shps_dev);
		return status;
	}

    acpi_entry = proc_create("bbswitch", 0664, acpi_root_dir, &bbswitch_fops);
    if (acpi_entry == NULL) {
        return -ENOMEM;
    }
	
	return 0;
}

static int sb2_shps_remove(struct platform_device *pdev)
{
	struct acpi_device *shps_dev = ACPI_COMPANION(&pdev->dev);

    remove_proc_entry("bbswitch", acpi_root_dir);
	acpi_dev_remove_driver_gpios(shps_dev);

	return 0;
}


static SIMPLE_DEV_PM_OPS(sb2_shps_pm_ops, NULL, sb2_shps_resume);

static const struct acpi_device_id sb2_shps_acpi_match[] = {
	{ "MSHW0153", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, sb2_shps_acpi_match);

static struct platform_driver sb2_shps_driver = {
	.probe = sb2_shps_probe,
	.remove = sb2_shps_remove,
	.driver = {
		.name = "sb2_shps",
		.acpi_match_table = ACPI_PTR(sb2_shps_acpi_match),
		.pm = &sb2_shps_pm_ops,
	},
};
module_platform_driver(sb2_shps_driver);

MODULE_AUTHOR("Maximilian Luz");
MODULE_DESCRIPTION("Surface Book 2 Hot-Plug System Driver");
MODULE_LICENSE("GPL v2");
