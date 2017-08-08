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
	{ 0xcd71858e, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x3356b90b, __VMLINUX_SYMBOL_STR(cpu_tss) },
	{ 0x71234250, __VMLINUX_SYMBOL_STR(class_remove_file_ns) },
	{ 0xf1669456, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x43ceeb3f, __VMLINUX_SYMBOL_STR(tty_unlock) },
	{ 0xedf578ce, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x6c09c2a4, __VMLINUX_SYMBOL_STR(del_timer) },
	{ 0x754d539c, __VMLINUX_SYMBOL_STR(strlen) },
	{ 0x39461d6a, __VMLINUX_SYMBOL_STR(in_egroup_p) },
	{ 0x8526c35a, __VMLINUX_SYMBOL_STR(remove_wait_queue) },
	{ 0xba40bf8e, __VMLINUX_SYMBOL_STR(tty_hung_up_p) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x2c903330, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0xfa78f0e3, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0x6729d3df, __VMLINUX_SYMBOL_STR(__get_user_4) },
	{ 0xeef47de9, __VMLINUX_SYMBOL_STR(tty_register_driver) },
	{ 0x9580deb, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0x440d0702, __VMLINUX_SYMBOL_STR(tty_buffer_request_room) },
	{ 0x92434ad6, __VMLINUX_SYMBOL_STR(put_tty_driver) },
	{ 0xb95600ef, __VMLINUX_SYMBOL_STR(tty_devnum) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x5454966f, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x11d43b39, __VMLINUX_SYMBOL_STR(tty_set_operations) },
	{ 0xe2d5255a, __VMLINUX_SYMBOL_STR(strcmp) },
	{ 0x9db9300a, __VMLINUX_SYMBOL_STR(proc_remove) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xffd5a395, __VMLINUX_SYMBOL_STR(default_wake_function) },
	{ 0xa8afb7c1, __VMLINUX_SYMBOL_STR(PDE_DATA) },
	{ 0xb8e7ce2c, __VMLINUX_SYMBOL_STR(__put_user_8) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xc42b8c14, __VMLINUX_SYMBOL_STR(proc_mkdir) },
	{ 0x11089ac7, __VMLINUX_SYMBOL_STR(_ctype) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0x8bdd5b5d, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0x51ae95ee, __VMLINUX_SYMBOL_STR(tty_ldisc_deref) },
	{ 0xf7b6fbdd, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0xfc6ed3d3, __VMLINUX_SYMBOL_STR(tty_ldisc_flush) },
	{ 0x93e5c759, __VMLINUX_SYMBOL_STR(tty_port_register_device) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0x542d1395, __VMLINUX_SYMBOL_STR(tty_port_init) },
	{ 0x8b8fa0f4, __VMLINUX_SYMBOL_STR(proc_mkdir_data) },
	{ 0x6dc6dd56, __VMLINUX_SYMBOL_STR(down) },
	{ 0x5419e3dc, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x1bb31047, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0x69fd128e, __VMLINUX_SYMBOL_STR(tty_ldisc_ref) },
	{ 0x1dc0ebe, __VMLINUX_SYMBOL_STR(sysfs_remove_link) },
	{ 0x542cdada, __VMLINUX_SYMBOL_STR(tty_insert_flip_string_flags) },
	{ 0x46849676, __VMLINUX_SYMBOL_STR(sysfs_create_link) },
	{ 0xcbf21372, __VMLINUX_SYMBOL_STR(module_put) },
	{ 0xc6cbbc89, __VMLINUX_SYMBOL_STR(capable) },
	{ 0x1d99aa51, __VMLINUX_SYMBOL_STR(tty_unregister_device) },
	{ 0xb2fd5ceb, __VMLINUX_SYMBOL_STR(__put_user_4) },
	{ 0xd784167b, __VMLINUX_SYMBOL_STR(tty_wait_until_sent) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0xa202a8e5, __VMLINUX_SYMBOL_STR(kmalloc_order_trace) },
	{ 0x6d334118, __VMLINUX_SYMBOL_STR(__get_user_8) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x26cd5e34, __VMLINUX_SYMBOL_STR(class_create_file_ns) },
	{ 0xab7d6a08, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0xf5b29531, __VMLINUX_SYMBOL_STR(tty_unregister_driver) },
	{ 0x6d529e99, __VMLINUX_SYMBOL_STR(tty_hangup) },
	{ 0xa6bbd805, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x2207a57f, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0x69266782, __VMLINUX_SYMBOL_STR(__tty_alloc_driver) },
	{ 0xaaee6d71, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0x4f68e5c9, __VMLINUX_SYMBOL_STR(do_gettimeofday) },
	{ 0xc9fef317, __VMLINUX_SYMBOL_STR(add_wait_queue) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xb1b1753e, __VMLINUX_SYMBOL_STR(tty_check_change) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0x138cda43, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0xf08242c2, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0x2b7f3d47, __VMLINUX_SYMBOL_STR(tty_flip_buffer_push) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x7af567e4, __VMLINUX_SYMBOL_STR(tty_wakeup) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0x962b781d, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x2482e688, __VMLINUX_SYMBOL_STR(vsprintf) },
	{ 0xbcddedf, __VMLINUX_SYMBOL_STR(tty_lock) },
	{ 0xe390f06e, __VMLINUX_SYMBOL_STR(try_module_get) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "51280132E5CAC242A1479B1");
