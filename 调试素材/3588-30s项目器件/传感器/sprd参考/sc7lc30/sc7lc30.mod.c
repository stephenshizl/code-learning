#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xc06c318c, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x572d0775, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x63648e73, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0xdcb764ad, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x84bc974b, __VMLINUX_SYMBOL_STR(__arch_copy_from_user) },
	{ 0xb35dea8f, __VMLINUX_SYMBOL_STR(__arch_copy_to_user) },
	{ 0x887568aa, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0xfcec0987, __VMLINUX_SYMBOL_STR(enable_irq) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x4aacd53e, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xce2840e7, __VMLINUX_SYMBOL_STR(irq_set_irq_wake) },
	{ 0x5e38de65, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x3914e87d, __VMLINUX_SYMBOL_STR(hrtimer_start_range_ns) },
	{ 0x65fb5dc5, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x54fdc586, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0xf76d9f1a, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x7492e364, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x3ce4ca6f, __VMLINUX_SYMBOL_STR(disable_irq) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xb59898b9, __VMLINUX_SYMBOL_STR(gpiod_direction_input) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0xf23fe2, __VMLINUX_SYMBOL_STR(gpiod_to_irq) },
	{ 0x33340419, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x99b2ebcf, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x64237665, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x2df5a899, __VMLINUX_SYMBOL_STR(misc_register) },
	{ 0xf9790e69, __VMLINUX_SYMBOL_STR(hrtimer_init) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0x121946fd, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0xb0d3cb4e, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0x4e87a1a5, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
	{ 0xae8c4d0c, __VMLINUX_SYMBOL_STR(set_bit) },
	{ 0xcbe708, __VMLINUX_SYMBOL_STR(input_allocate_device) },
	{ 0x5863352d, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0xbe4f4439, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x965702de, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0xe2eb0ddf, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xdce4e9fc, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xc647ee87, __VMLINUX_SYMBOL_STR(hrtimer_cancel) },
	{ 0x85df9b6c, __VMLINUX_SYMBOL_STR(strsep) },
	{ 0xf32a9dfc, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0x9f87f526, __VMLINUX_SYMBOL_STR(hrtimer_forward) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x27bbf221, __VMLINUX_SYMBOL_STR(disable_irq_nosync) },
	{ 0x4ca9669f, __VMLINUX_SYMBOL_STR(scnprintf) },
	{ 0x60ea2d6, __VMLINUX_SYMBOL_STR(kstrtoull) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x495a40df, __VMLINUX_SYMBOL_STR(nonseekable_open) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0xa6fd933c, __VMLINUX_SYMBOL_STR(hrtimer_try_to_cancel) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0xc1514a3b, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0x78ddedae, __VMLINUX_SYMBOL_STR(misc_deregister) },
	{ 0xa39b1027, __VMLINUX_SYMBOL_STR(device_init_wakeup) },
	{ 0x8f678b07, __VMLINUX_SYMBOL_STR(__stack_chk_guard) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0x4a451dee, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:Silan_alsps");

MODULE_INFO(srcversion, "9EB6FBA923DC584B24BDC88");
