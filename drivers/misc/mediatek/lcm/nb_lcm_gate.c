/* Add by NewBund dengwenhao,  for lcm_gate driver. 
  LP3101 default5.4v and KTD2151 default 5v	,we only need 5.4v*/

/*------LCM Gate Drive  AVDD&AVEE 5.4v-----*/


#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/i2c.h>



static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};
#define I2C_ID_NAME "lcm_gate"


static struct i2c_client *lcm_gate_i2c_client;


/*****************************************************************************
* Function Prototype
*****************************************************************************/
static int lcm_gate_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lcm_gate_remove(struct i2c_client *client);
/*****************************************************************************
* Data Structure
*****************************************************************************/

struct lcm_gate_dev	{
struct i2c_client	*client;

};

static const struct i2c_device_id lcm_gate_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

/*
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif
*/
static struct i2c_driver lcm_gate_iic_driver = {
	.id_table	= lcm_gate_id,
	.probe		= lcm_gate_probe,
	.remove		= lcm_gate_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "lcm_gate",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
};
/*****************************************************************************
* Extern Area
*****************************************************************************/

/*****************************************************************************
* Function
*****************************************************************************/
static int lcm_gate_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_debug("lcm_gate_iic_probe\n");
	pr_debug("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	lcm_gate_i2c_client  = client;
	return 0;
}


static int lcm_gate_remove(struct i2c_client *client)
{
	pr_debug("lcm_gate_remove\n");
	lcm_gate_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


int lcm_gate_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = lcm_gate_i2c_client;
	char write_data[2] = {0};

	if (client == NULL) {
		pr_debug("ERROR!!lcm_gate_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_debug("lcm_gate write data fail !!\n");
	return ret;
}
EXPORT_SYMBOL_GPL(lcm_gate_write_bytes);


/*
* module load/unload record keeping
*/

static int __init lcm_gate_iic_init(void)
{
	pr_debug("lcm_gate_iic_init\n");
	i2c_add_driver(&lcm_gate_iic_driver);
	pr_debug("lcm_gate_iic_init success\n");
	return 0;
}

static void __exit lcm_gate_iic_exit(void)
{
	pr_debug("lcm_gate_iic_exit\n");
	i2c_del_driver(&lcm_gate_iic_driver);
}

module_init(lcm_gate_iic_init);
module_exit(lcm_gate_iic_exit);




