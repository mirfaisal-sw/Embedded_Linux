#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

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

MODULE_INFO(intree, "Y");

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x9e2dc0be, "module_layout" },
	{ 0xcf56d7c4, "param_ops_int" },
	{ 0xae4f420b, "no_llseek" },
	{ 0xc0e313cd, "driver_unregister" },
	{ 0x3764c28e, "__spi_register_driver" },
	{ 0x11a32253, "kmalloc_caches" },
	{ 0x676bbc0f, "_set_bit" },
	{ 0xd42ab1b7, "device_create" },
	{ 0xfe90c4a6, "_find_first_zero_bit_le" },
	{ 0xdd978dea, "__class_create" },
	{ 0x513aec5a, "__register_chrdev" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x496b588f, "device_property_read_u32_array" },
	{ 0xab9cf504, "kmem_cache_alloc_trace" },
	{ 0x28cc25db, "arm_copy_from_user" },
	{ 0xb70789e, "__might_fault" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0x5f19b8c8, "spi_sync" },
	{ 0x5f754e5a, "memset" },
	{ 0x37a0cba, "kfree" },
	{ 0x49ebacbd, "_clear_bit" },
	{ 0xa5d2e60, "device_destroy" },
	{ 0xbde8da43, "pm_runtime_allow" },
	{ 0x7c32d0f0, "printk" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0x9758bcac, "mutex_unlock" },
	{ 0x3a3fab95, "mutex_lock_nested" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("spi:mirflash");
MODULE_ALIAS("of:N*T*Cmir,mirflash");
MODULE_ALIAS("of:N*T*Cmir,mirflashC*");

MODULE_INFO(srcversion, "D9B7E9BC6A93591EFA3F41F");
