#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x25701227, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x5a8b8db3, __VMLINUX_SYMBOL_STR(class_remove_file_ns) },
	{ 0xa1a14995, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x4c4fef19, __VMLINUX_SYMBOL_STR(kernel_stack) },
	{ 0x36c9001b, __VMLINUX_SYMBOL_STR(tty_unlock) },
	{ 0xb6b46a7c, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xc996d097, __VMLINUX_SYMBOL_STR(del_timer) },
	{ 0x754d539c, __VMLINUX_SYMBOL_STR(strlen) },
	{ 0x39461d6a, __VMLINUX_SYMBOL_STR(in_egroup_p) },
	{ 0xe00b7e06, __VMLINUX_SYMBOL_STR(dev_set_drvdata) },
	{ 0xb5dcab5b, __VMLINUX_SYMBOL_STR(remove_wait_queue) },
	{ 0x630ebac4, __VMLINUX_SYMBOL_STR(tty_hung_up_p) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x3abb024e, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0x559f5aa, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0x6729d3df, __VMLINUX_SYMBOL_STR(__get_user_4) },
	{ 0xc50fc0da, __VMLINUX_SYMBOL_STR(tty_register_driver) },
	{ 0x593a99b, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0xba71627f, __VMLINUX_SYMBOL_STR(tty_buffer_request_room) },
	{ 0xaca3da28, __VMLINUX_SYMBOL_STR(put_tty_driver) },
	{ 0x4a4f9ac1, __VMLINUX_SYMBOL_STR(tty_devnum) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x23adc62a, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x27802a0, __VMLINUX_SYMBOL_STR(tty_set_operations) },
	{ 0xe2d5255a, __VMLINUX_SYMBOL_STR(strcmp) },
	{ 0x740d118f, __VMLINUX_SYMBOL_STR(proc_remove) },
	{ 0xf432dd3d, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xffd5a395, __VMLINUX_SYMBOL_STR(default_wake_function) },
	{ 0xd93b1cc8, __VMLINUX_SYMBOL_STR(PDE_DATA) },
	{ 0xb8e7ce2c, __VMLINUX_SYMBOL_STR(__put_user_8) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x89428114, __VMLINUX_SYMBOL_STR(proc_mkdir) },
	{ 0x11089ac7, __VMLINUX_SYMBOL_STR(_ctype) },
	{ 0x8f64aa4, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0xb11aac9e, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0x9907d1e8, __VMLINUX_SYMBOL_STR(tty_ldisc_deref) },
	{ 0x6e3311de, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0xc4fb6770, __VMLINUX_SYMBOL_STR(tty_ldisc_flush) },
	{ 0x931e5fd4, __VMLINUX_SYMBOL_STR(tty_port_register_device) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0xf5969736, __VMLINUX_SYMBOL_STR(tty_port_init) },
	{ 0xa09398f3, __VMLINUX_SYMBOL_STR(proc_mkdir_data) },
	{ 0x68aca4ad, __VMLINUX_SYMBOL_STR(down) },
	{ 0x5c5c9ac4, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0xbe2c0274, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0x388ae639, __VMLINUX_SYMBOL_STR(tty_ldisc_ref) },
	{ 0xcdcfe6c4, __VMLINUX_SYMBOL_STR(sysfs_remove_link) },
	{ 0xda4c5b67, __VMLINUX_SYMBOL_STR(tty_insert_flip_string_flags) },
	{ 0x3d88cad5, __VMLINUX_SYMBOL_STR(sysfs_create_link) },
	{ 0x5235b0ca, __VMLINUX_SYMBOL_STR(module_put) },
	{ 0xc6cbbc89, __VMLINUX_SYMBOL_STR(capable) },
	{ 0xf8c3ab52, __VMLINUX_SYMBOL_STR(tty_unregister_device) },
	{ 0xb2fd5ceb, __VMLINUX_SYMBOL_STR(__put_user_4) },
	{ 0x64089be, __VMLINUX_SYMBOL_STR(tty_wait_until_sent) },
	{ 0xf0fdf6cb, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{        0, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0xa202a8e5, __VMLINUX_SYMBOL_STR(kmalloc_order_trace) },
	{ 0x6d334118, __VMLINUX_SYMBOL_STR(__get_user_8) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xbe8b39a4, __VMLINUX_SYMBOL_STR(class_create_file_ns) },
	{ 0xbad3ae94, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x9327f5ce, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0xcdeb9b3a, __VMLINUX_SYMBOL_STR(tty_unregister_driver) },
	{ 0xe19970b8, __VMLINUX_SYMBOL_STR(tty_hangup) },
	{ 0xcf21d241, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x34f22f94, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0x46f68fd8, __VMLINUX_SYMBOL_STR(__tty_alloc_driver) },
	{ 0x64f47ab0, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0x4f68e5c9, __VMLINUX_SYMBOL_STR(do_gettimeofday) },
	{ 0x5860aad4, __VMLINUX_SYMBOL_STR(add_wait_queue) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x3b38d411, __VMLINUX_SYMBOL_STR(tty_check_change) },
	{ 0x71e3cecb, __VMLINUX_SYMBOL_STR(up) },
	{ 0x2b08fecb, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0xfa66f77c, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0xaa9787d0, __VMLINUX_SYMBOL_STR(tty_flip_buffer_push) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xa64c16de, __VMLINUX_SYMBOL_STR(tty_wakeup) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0xb4b7177e, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x2482e688, __VMLINUX_SYMBOL_STR(vsprintf) },
	{ 0x479fe2c9, __VMLINUX_SYMBOL_STR(dev_get_drvdata) },
	{ 0xa2286070, __VMLINUX_SYMBOL_STR(tty_lock) },
	{ 0x18e1f3d1, __VMLINUX_SYMBOL_STR(try_module_get) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "51280132E5CAC242A1479B1");
