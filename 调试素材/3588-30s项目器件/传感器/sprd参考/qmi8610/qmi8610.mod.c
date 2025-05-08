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
	{ 0x9b2a5cee, __VMLINUX_SYMBOL_STR(__pm_relax) },
	{ 0xb2216777, __VMLINUX_SYMBOL_STR(__pm_stay_awake) },
	{ 0xae982088, __VMLINUX_SYMBOL_STR(wake_up_process) },
	{ 0x1118790c, __VMLINUX_SYMBOL_STR(kthread_create_on_node) },
	{ 0xdcb764ad, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x84bc974b, __VMLINUX_SYMBOL_STR(__arch_copy_from_user) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0x6b06fdce, __VMLINUX_SYMBOL_STR(delayed_work_timer_fn) },
	{ 0x65fb5dc5, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x99b2ebcf, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0xa5942e0b, __VMLINUX_SYMBOL_STR(input_unregister_device) },
	{ 0x78ddedae, __VMLINUX_SYMBOL_STR(misc_deregister) },
	{ 0x5ee52022, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0x64237665, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x121946fd, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0x2df5a899, __VMLINUX_SYMBOL_STR(misc_register) },
	{ 0xb0d3cb4e, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0x4e87a1a5, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
	{ 0x7c451aa7, __VMLINUX_SYMBOL_STR(input_set_capability) },
	{ 0xcbe708, __VMLINUX_SYMBOL_STR(input_allocate_device) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0xe4f011aa, __VMLINUX_SYMBOL_STR(gpiod_direction_output_raw) },
	{ 0x33340419, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x5863352d, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0xbe4f4439, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xb2213161, __VMLINUX_SYMBOL_STR(of_property_read_variable_u32_array) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xe2eb0ddf, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xdce4e9fc, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x5d67767d, __VMLINUX_SYMBOL_STR(wakeup_source_add) },
	{ 0x728b3e87, __VMLINUX_SYMBOL_STR(wakeup_source_prepare) },
	{ 0xf32a9dfc, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x4ca9669f, __VMLINUX_SYMBOL_STR(scnprintf) },
	{ 0x4a451dee, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x75090002, __VMLINUX_SYMBOL_STR(queue_delayed_work_on) },
	{ 0x7f02188f, __VMLINUX_SYMBOL_STR(__msecs_to_jiffies) },
	{ 0x64af6747, __VMLINUX_SYMBOL_STR(cancel_delayed_work_sync) },
	{ 0xe914e41e, __VMLINUX_SYMBOL_STR(strcpy) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x8f678b07, __VMLINUX_SYMBOL_STR(__stack_chk_guard) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0x4aacd53e, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x72ab4204, __VMLINUX_SYMBOL_STR(i2c_smbus_write_byte_data) },
	{ 0x5e38de65, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x495a40df, __VMLINUX_SYMBOL_STR(nonseekable_open) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cqst,qmi8610");
MODULE_ALIAS("of:N*T*Cqst,qmi8610C*");
MODULE_ALIAS("i2c:qmi8610");

MODULE_INFO(srcversion, "BD9E80D2D5E3423A5533E70");
