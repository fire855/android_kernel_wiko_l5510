
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <mach/gpiomux.h>
#include <linux/wakelock.h>

struct hall_platform_data {
	int gpio_en;
	int gpio_int;
	u32 int_flags;
	bool use_int;
};

struct hall_ic_t {
	int irq;
	int gpio_int;
	bool power_enabled;
	//struct platform_device *pdev;
	struct hall_platform_data *pdata;
	//struct regulator *vi2c;
	struct mutex lock;	
	struct wake_lock	hall_wake_lock;
};


//#define PINCTRL_STATE_ACTIVE	"hall_ic_active"
//#define PINCTRL_STATE_SUSPEND	"hall_ic_suspend"

//#define HALL_VI2C_MIN_UV	1800000
//#define HALL_VI2C_MAX_UV	1800000

#define HALL_DEBUG
static struct hall_ic_t  *ghall_ic;
//static int hall_value = 0;
static int hall_data = 0;

#if 0
static ssize_t hall_value_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int var;
	if(ghall_ic == NULL)
		return 0;
	
	mutex_lock(&ghall_ic->lock);
	var = hall_value;
	mutex_unlock(&ghall_ic->lock);		
	
	pr_info("hall value : %d\n", var);
	return sprintf(buf, "%d\n", var);
}


static ssize_t hall_value_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int var;
	sscanf(buf, "%du", &var);
	hall_value = var;
	return count;
}


//change 0666 to 0664 for CTS 
//static struct kobj_attribute hall_value_attribute = __ATTR(hall_value, 0666, hall_value_show, NULL);
static struct kobj_attribute hall_value_attribute = __ATTR(hall_value, 0664, hall_value_show, NULL);

static struct attribute *attrs[] = {
	&hall_value_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *hall_ic_kobj;
#endif

static ssize_t show_hall_data(struct device *dev,struct device_attribute *attr, char *buf)
{
#ifdef HALL_DEBUG
    pr_info(" show_hall_data : %d \n", hall_data);
#endif

    return sprintf(buf, "%u\n", hall_data);
}
static DEVICE_ATTR(hall_data, 0664, show_hall_data, NULL);



#ifdef CONFIG_OF
static int hall_parse_dt(struct device *dev, struct hall_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_int = of_get_named_gpio_flags(np, "qcom,irq-gpio", 0, &pdata->int_flags);

	return 0;
}
#else
static int hall_parse_dt(struct device *dev, struct hall_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static irqreturn_t hall_interrupt_thread(int irq, void *data)
{
	struct hall_ic_t *hall_ic = data;
	int val = gpio_get_value_cansleep(ghall_ic->gpio_int);
	
	mutex_lock(&hall_ic->lock);
	#ifdef HALL_DEBUG
	pr_info("%s : hall_data pre = %d    irq = %d   val = %d  \n", __func__, hall_data ,irq ,val);
	#endif 
	//if(hall_data == 0)  // !hall_value;
		//hall_data = 1;
	//else
	hall_data = val;
	//pr_info("%s : hall_data next =    %d\n", __func__, hall_data);
	mutex_unlock(&hall_ic->lock);	
       wake_lock_timeout(&ghall_ic->hall_wake_lock, HZ * 2);
	return IRQ_HANDLED;
}

static int hall_ic_probe(struct platform_device *pdev)
{
	int ret;
	struct hall_platform_data *pdata;
	struct device *dev = &pdev->dev;
	int ret_device_file = 0;
	
	ghall_ic = kzalloc(sizeof(struct hall_ic_t), GFP_KERNEL);
	if (!ghall_ic) {
		pr_err("%s : Failed to allocate driver data\n", __func__);
		return -ENOMEM;
	}

	if (dev->of_node) {
		pdata = kzalloc(sizeof(struct hall_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s : Failed to allcated memory\n", __func__);
			ret = -ENOMEM;
			goto err_free_devmem;
		}
		ret = hall_parse_dt(dev, pdata);
		if (ret) {
			pr_err("%s : Failed to parse device tree\n", __func__);
			ret = -EINVAL;
			goto err_free_pdatamem;
		}
	} else {
		pdata = dev->platform_data;
		if (!pdata) {
			pr_err("%s : Cannot get device platform data\n", __func__);
			ret = -EINVAL;
			goto err_free_pdatamem;
		}
	}

	ret_device_file = device_create_file(dev, &dev_attr_hall_data);

	mutex_init(&ghall_ic->lock);
	wake_lock_init(&ghall_ic->hall_wake_lock, WAKE_LOCK_SUSPEND,
			"android-hallchange");
	
	//ghall_ic->pdev = pdev;
	ghall_ic->pdata = pdata;
	//platform_set_drvdata(pdev, ghall_ic);

	ghall_ic->gpio_int = ghall_ic->pdata->gpio_int;
	if (gpio_is_valid(ghall_ic->gpio_int)) {
		ret = gpio_request(ghall_ic->gpio_int, "hall_gpio_int");
		if(ret < 0){
			pr_err("%s : Failed to request GPIO=%d, ret=%d\n", __func__, ghall_ic->gpio_int, ret);
			ret = -EINVAL;
			goto err_free_pdatamem;			
		}
		
		ret = gpio_direction_input(ghall_ic->gpio_int);
		if(ret < 0){
			pr_err("%s : Failed to set GPIO=%d, ret=%d\n", __func__, ghall_ic->gpio_int, ret);
			ret = -EINVAL;
			goto err_free_pdatamem;			
		}		
	}

	ghall_ic->irq = gpio_to_irq(ghall_ic->gpio_int);
	ret = request_irq(ghall_ic->irq, hall_interrupt_thread, 
			     IRQF_TRIGGER_RISING |
			     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,	
			"hall_ic", ghall_ic);
	
	if (ret) {
		pr_err("%s : Can't get IRQ %d, error %d\n", __func__, ghall_ic->irq, ret);
		goto err_free_irq_gpio;
	}
	else{
		enable_irq_wake(ghall_ic->irq);
		hall_data = gpio_get_value_cansleep(ghall_ic->gpio_int);
	}

	return ret;

err_free_irq_gpio:
	if (gpio_is_valid(ghall_ic->gpio_int))
		gpio_free(ghall_ic->gpio_int);
err_free_pdatamem:
	kfree(pdata);	

err_free_devmem:
	kfree(ghall_ic);
	wake_lock_destroy(&ghall_ic->hall_wake_lock);
	pr_err("%s : Probe device return error%d\n", __func__, ret);
	return ret;
}
static int hall_ic_remove(struct platform_device *pdev)
{
	struct hall_ic_t *hall_ic = platform_get_drvdata(pdev);

	free_irq(hall_ic->irq, hall_ic);
	//misc_deregister(&hall_ic->qca199x_device);
	gpio_free(hall_ic->gpio_int);
	wake_lock_destroy(&ghall_ic->hall_wake_lock);
	kfree(hall_ic);

	return 0;
}

static struct of_device_id hall_match_table[] = {
	{.compatible = "hall-ic"},
	{}
};

#if 0//def CONFIG_PM_SLEEP
static int hall_suspend(struct device *dev)
{
#if 0
	struct hall_ic_t *hall_ic;
	int ret = 0;
	
	pr_info("%s : enter.\n", __func__);

	hall_ic = dev_get_drvdata(dev);
	
	if (hall_ic->hall_pinctrl) {
		ret = pinctrl_select_state(ghall_ic->hall_pinctrl, ghall_ic->pinctrl_state_suspend);
		if (ret < 0) {
			pr_err("%s : failed to select pin to active state, ret=%d\n", __func__, ret);
		}
	}
	ret = hall_power_ctl(ghall_ic, false);
	if (ret) {
		pr_err("%s : Failed to power on device,ret=%d\n", __func__, ret);
	}
	enable_irq_wake(hall_ic->irq);
#endif
	return 0;
}

static int hall_resume(struct device *dev)
{

	struct hall_ic_t *hall_ic;
	int ret = 0;

	pr_info("%s : enter.\n", __func__);
	
	hall_ic = dev_get_drvdata(dev);
	
	if (hall_ic->hall_pinctrl) {
		ret = pinctrl_select_state(ghall_ic->hall_pinctrl, ghall_ic->pinctrl_state_active);
		if (ret < 0) {
			pr_err("%s : failed to select pin to active state, ret=%d\n", __func__, ret);
		}
	}
	
	ret = hall_power_ctl(ghall_ic, true);
	if (ret) {
		pr_err("%s : Failed to power on device,ret=%d\n", __func__, ret);
	}
	disable_irq_wake(hall_ic->irq);

	return 0;
}
#else
static int hall_suspend(struct device *dev)
{
	return 0;
}
static int hall_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(hall_pm_ops, hall_suspend, hall_resume);

static struct platform_driver hall_ic_driver = {
	.probe = hall_ic_probe,
	.remove = hall_ic_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "hall-ic",
		.pm	= &hall_pm_ops,
		.of_match_table = of_match_ptr(hall_match_table),
	},
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_ic_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_ic_driver);
	//kobject_put(hall_ic_kobj);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("tinno");
