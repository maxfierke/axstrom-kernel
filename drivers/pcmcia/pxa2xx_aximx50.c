/*
 * linux/drivers/pcmcia/pxa2xx_aximx50.c
 *
 * PCMCIA Support for Dell Axim X50(v)/X51(v)
 *
 * Copyright(C) 2010 Max Fierke <max@maxfierke.com>
 * Copyright(C) 2010 Paul Burton <paulburton89@gmail.com>
 *
 * Based on pxa2xx_mainstone.c and aximx50_pcmcia.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <pcmcia/ss.h>

#include <asm/mach-types.h>
#include <asm/irq.h>

#include <mach/pxa2xx-regs.h>
#include <mach/aximx50.h>
#include <mach/gpio.h>

#include "soc_common.h"

static struct pcmcia_irqs irqs[] = {
	{ 0, GPIO_NR_AXIMX50_PCMCIA_DETECT1, "PCMCIA1 CD" },
};

static int aximx50_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	int ret;

	if (skt->nr < 0 || skt->nr > 1)
		return -ENODEV;

	/* WiFi is handled elsewhere */
	if (skt->nr == 0)
		return 0;

	ret = gpio_request(GPIO_NR_AXIMX50_PCMCIA_DETECT1, "PCMCIA DET");
	if (ret)
		goto err1;
	ret = gpio_direction_input(GPIO_NR_AXIMX50_PCMCIA_DETECT1);
	if (ret)
		goto err2;

	ret = gpio_request(GPIO_NR_AXIMX50_PCMCIA_RESET1, "PCMCIA RST");
	if (ret)
		goto err2;
	ret = gpio_direction_output(GPIO_NR_AXIMX50_PCMCIA_RESET1, 1);
	if (ret)
		goto err3;

	ret = gpio_request(GPIO_NR_AXIMX50_PCMCIA_READY1, "PCMCIA RDY");
	if (ret)
		goto err3;
	ret = gpio_direction_input(GPIO_NR_AXIMX50_PCMCIA_READY1);
	if (ret)
		goto err4;

	skt->socket.pci_irq = IRQ_GPIO(GPIO_NR_AXIMX50_PCMCIA_READY1);

	ret = soc_pcmcia_request_irqs(skt, irqs, ARRAY_SIZE(irqs));
	if (ret)
		goto err4;

	return 0;

err4:
	gpio_free(GPIO_NR_AXIMX50_PCMCIA_READY1);
err3:
	gpio_free(GPIO_NR_AXIMX50_PCMCIA_RESET1);
err2:
	gpio_free(GPIO_NR_AXIMX50_PCMCIA_DETECT1);
err1:
	return ret;
}

static void aximx50_pcmcia_hw_shutdown(struct soc_pcmcia_socket *skt)
{
	soc_pcmcia_free_irqs(skt, irqs, ARRAY_SIZE(irqs));
}

static void aximx50_pcmcia_socket_state(struct soc_pcmcia_socket *skt, struct pcmcia_state *state)
{
	if (skt->nr == 0) {
		/* WiFi is handled elsewhere */
		state->detect = 1;
		state->ready = 1;
	}
	else {
		state->detect = !!gpio_get_value(GPIO_NR_AXIMX50_PCMCIA_DETECT1);
		state->ready = !!gpio_get_value(GPIO_NR_AXIMX50_PCMCIA_READY1);
	}

	state->bvd1   = 1;
	state->bvd2   = 1;
	state->vs_3v  = 1;
	state->vs_Xv  = 0;
	state->wrprot = 0;
}

static int aximx50_pcmcia_configure_socket(struct soc_pcmcia_socket *skt, const socket_state_t *state)
{
	int ret = 0;

	if (state->flags & SS_RESET) {
		if (skt->nr == 0)
			{ /* WiFi */ }
		else
			gpio_set_value(GPIO_NR_AXIMX50_PCMCIA_RESET1, 1);
	}
	else {
		if (skt->nr == 0)
			{ /* WiFi */ }
		else
			gpio_set_value(GPIO_NR_AXIMX50_PCMCIA_RESET1, 0);
	}

	/* Apply socket voltage */
	switch (state->Vcc) {
	case 0:
		if (skt->nr == 0) {
			/* WiFi */
		}
		else {
			aximx50_fpga_clear(0x10, 0x1000);
			aximx50_fpga_clear(0x16, 0x0002);
		}

		break;

	case 50:
	case 33:
		if (skt->nr == 0) {
			/* WiFi */
		}
		else {
			aximx50_fpga_set(0x10, 0x1000);
			aximx50_fpga_set(0x16, 0x0002);
		}

		break;

	default:
		printk(KERN_ERR "%s: Unsupported Vcc:%d\n", __FUNCTION__, state->Vcc);
		ret = -1;
		break;
	}

	return ret;
}

static void aximx50_pcmcia_socket_init(struct soc_pcmcia_socket *skt)
{
}

static void aximx50_pcmcia_socket_suspend(struct soc_pcmcia_socket *skt)
{
}

static struct pcmcia_low_level aximx50_pcmcia_ops = {
	.owner            = THIS_MODULE,

	.hw_init          = aximx50_pcmcia_hw_init,
	.hw_shutdown	  = aximx50_pcmcia_hw_shutdown,
	.socket_state	  = aximx50_pcmcia_socket_state,
	.configure_socket = aximx50_pcmcia_configure_socket,
	.socket_init	  = aximx50_pcmcia_socket_init,
	.socket_suspend	  = aximx50_pcmcia_socket_suspend,

	.nr               = 2,
};

static struct platform_device *aximx50_pcmcia_device;

static int __init aximx50_pcmcia_init(void)
{
	int ret;

	aximx50_pcmcia_device = platform_device_alloc("pxa2xx-pcmcia", -1);
	if (!aximx50_pcmcia_device)
		return -ENOMEM;

	ret = platform_device_add_data(aximx50_pcmcia_device, &aximx50_pcmcia_ops,
	                               sizeof(aximx50_pcmcia_ops));

	if (!ret)
		ret = platform_device_add(aximx50_pcmcia_device);

	if (ret)
		platform_device_put(aximx50_pcmcia_device);

	return ret;
}

static void __exit aximx50_pcmcia_exit(void)
{
	platform_device_unregister(aximx50_pcmcia_device);
}

module_init(aximx50_pcmcia_init);
module_exit(aximx50_pcmcia_exit);

MODULE_AUTHOR("Max Fierke <max@maxfierke.com>, Paul Burton <paulburton89@gmail.com>");
MODULE_DESCRIPTION("Dell Axim X50(v)/X51(v) PCMCIA Support");
MODULE_LICENSE("GPLv2");

