/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>
 
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
 
#include <zephyr/pm/state.h>
 
 
#define SD_SPI3_SCK_PIN   6
#define SD_SPI3_MOSI_PIN  7
#define SD_SPI3_MISO_PIN  5
#define SD_SPI3_CS_PIN    3
 
#define DEVICE_PM_ACTIVE_STATE      0
#define DEVICE_PM_LOW_POWER_STATE   1
#define DEVICE_PM_SUSPEND_STATE     2
#define DEVICE_PM_OFF_STATE         3
 
 
LOG_MODULE_REGISTER(main);
 
#if CONFIG_DISK_DRIVER_FLASH
#include <zephyr/storage/flash_map.h>
#endif
 
#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#endif
 
#if CONFIG_FILE_SYSTEM_LITTLEFS
#include <zephyr/fs/littlefs.h>
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
#endif
 
#define STORAGE_PARTITION       storage_partition
#define STORAGE_PARTITION_ID        FIXED_PARTITION_ID(STORAGE_PARTITION)
 
static struct fs_mount_t fs_mnt;
 
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_CONFIGURATION_DEFINE(config_1,
              USB_SCD_SELF_POWERED,
              200);
 
USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "ZEPHYR");
USBD_DESC_PRODUCT_DEFINE(sample_product, "Zephyr USBD MSC");
USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn, "0123456789ABCDEF");
 
 
USBD_DEVICE_DEFINE(sample_usbd,
           DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
           0x2fe3, 0x0008);
 
#if CONFIG_DISK_DRIVER_RAM
USBD_DEFINE_MSC_LUN(RAM, "Zephyr", "RAMDisk", "0.00");
#endif
 
#if CONFIG_DISK_DRIVER_FLASH
USBD_DEFINE_MSC_LUN(NAND, "Zephyr", "FlashDisk", "0.00");
#endif
 
#if CONFIG_DISK_DRIVER_SDMMC
USBD_DEFINE_MSC_LUN(SD, "Zephyr", "SD", "0.00");
#endif
 
static int enable_usb_device_next(void)
{
    int err;
 
    err = usbd_add_descriptor(&sample_usbd, &sample_lang);
    if (err) {
        LOG_ERR("Failed to initialize language descriptor (%d)", err);
        return err;
    }
 
    err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
    if (err) {
        LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
        return err;
    }
 
    err = usbd_add_descriptor(&sample_usbd, &sample_product);
    if (err) {
        LOG_ERR("Failed to initialize product descriptor (%d)", err);
        return err;
    }
 
    err = usbd_add_descriptor(&sample_usbd, &sample_sn);
    if (err) {
        LOG_ERR("Failed to initialize SN descriptor (%d)", err);
        return err;
    }
 
    err = usbd_add_configuration(&sample_usbd, &config_1);
    if (err) {
        LOG_ERR("Failed to add configuration (%d)", err);
        return err;
    }
 
    err = usbd_register_class(&sample_usbd, "msc_0", 1);
    if (err) {
        LOG_ERR("Failed to register MSC class (%d)", err);
        return err;
    }
 
    err = usbd_init(&sample_usbd);
    if (err) {
        LOG_ERR("Failed to initialize device support");
        return err;
    }
 
    err = usbd_enable(&sample_usbd);
    if (err) {
        LOG_ERR("Failed to enable device support");
        return err;
    }
 
    LOG_DBG("USB device support enabled");
 
    return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */
 
static int setup_flash(struct fs_mount_t *mnt)
{
    int rc = 0;
#if CONFIG_DISK_DRIVER_FLASH
    unsigned int id;
    const struct flash_area *pfa;
 
    mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
    id = STORAGE_PARTITION_ID;
 
    rc = flash_area_open(id, &pfa);
    printk("Area %u at 0x%x on %s for %u bytes\n",
           id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
           (unsigned int)pfa->fa_size);
 
    if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
        printk("Erasing flash area ... ");
        rc = flash_area_erase(pfa, 0, pfa->fa_size);
        printk("%d\n", rc);
    }
 
    if (rc < 0) {
        flash_area_close(pfa);
    }
#endif
    return rc;
}
 
static int mount_app_fs(struct fs_mount_t *mnt)
{
    int rc;
 
#if CONFIG_FAT_FILESYSTEM_ELM
    static FATFS fat_fs;
 
    mnt->type = FS_FATFS;
    mnt->fs_data = &fat_fs;
    if (IS_ENABLED(CONFIG_DISK_DRIVER_RAM)) {
        mnt->mnt_point = "/RAM:";
    } else if (IS_ENABLED(CONFIG_DISK_DRIVER_SDMMC)) {
        mnt->mnt_point = "/SD:";
    } else {
        mnt->mnt_point = "/NAND:";
    }
 
#elif CONFIG_FILE_SYSTEM_LITTLEFS
    mnt->type = FS_LITTLEFS;
    mnt->mnt_point = "/lfs";
    mnt->fs_data = &storage;
#endif
    rc = fs_mount(mnt);
 
    return rc;
}
 
static void setup_disk(void)
{
    struct fs_mount_t *mp = &fs_mnt;
    struct fs_dir_t dir;
    struct fs_statvfs sbuf;
    int rc;
 
    fs_dir_t_init(&dir);
 
    if (IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
        rc = setup_flash(mp);
        if (rc < 0) {
            LOG_ERR("Failed to setup flash area");
            return;
        }
    }
 
    if (!IS_ENABLED(CONFIG_FILE_SYSTEM_LITTLEFS) &&
        !IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
        LOG_INF("No file system selected");
        return;
    }
 
    rc = mount_app_fs(mp);
    if (rc < 0) {
        LOG_ERR("Failed to mount filesystem");
        return;
    }
 
    /* Allow log messages to flush to avoid interleaved output */
    k_sleep(K_MSEC(50));
 
    printk("Mount %s: %d\n", fs_mnt.mnt_point, rc);
 
    rc = fs_statvfs(mp->mnt_point, &sbuf);
    if (rc < 0) {
        printk("FAIL: statvfs: %d\n", rc);
        return;
    }
 
    printk("%s: bsize = %lu ; frsize = %lu ;"
           " blocks = %lu ; bfree = %lu\n",
           mp->mnt_point,
           sbuf.f_bsize, sbuf.f_frsize,
           sbuf.f_blocks, sbuf.f_bfree);
 
    rc = fs_opendir(&dir, mp->mnt_point);
    printk("%s opendir: %d\n", mp->mnt_point, rc);
 
    if (rc < 0) {
        LOG_ERR("Failed to open directory");
    }
 
    while (rc >= 0) {
        struct fs_dirent ent = { 0 };
 
        rc = fs_readdir(&dir, &ent);
        if (rc < 0) {
            LOG_ERR("Failed to read directory entries");
            break;
        }
        if (ent.name[0] == 0) {
            printk("End of files\n");
            break;
        }
        printk("  %c %u %s\n",
               (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
               ent.size,
               ent.name);
    }
 
    (void)fs_closedir(&dir);
 
    return;
}
 
#define TEST_FILE_PATH "/SD:/test2.bin"
#define READ_BUF_SIZE 65536  // 4 KB buffer
 
static void benchmark_file_read(const char *file_path)
{
    struct fs_file_t file;
    __aligned(4) static uint8_t buf[READ_BUF_SIZE];
    int rc;
    ssize_t read_bytes;
    uint32_t total_read = 0;
 
    fs_file_t_init(&file);
 
    printk("Opening file: %s\n", file_path);
    rc = fs_open(&file, file_path, FS_O_READ);
    if (rc < 0) {
        printk("Failed to open file (%d)\n", rc);
        return;
    }
 
    uint32_t start_time = k_uptime_get_32();
 
    while ((read_bytes = fs_read(&file, buf, sizeof(buf))) > 0) {
        total_read += read_bytes;
    }
 
    uint32_t end_time = k_uptime_get_32();
   
 
    fs_close(&file);
 
    if (read_bytes < 0) {
        printk("File read error: %d\n", read_bytes);
        return;
    }
 
    uint32_t elapsed_ms = end_time - start_time;
    uint32_t speed_kbps = total_read / elapsed_ms ;
 
    printf("Read %u bytes in %u ms => %u KB/s\n", total_read, elapsed_ms, speed_kbps);
 
}
 
static void spi3_power_cycle_test(void)
{
    const struct device *spi3_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
 
    if (!device_is_ready(spi3_dev)) {
        printk("SPI3 device not ready!\n");
        return;
    }
 
    while (1) {
        printk("\n=== SPI3 Power Cycle Test LOOP ===\n");
 
        printk("Running benchmark...\n");
        benchmark_file_read(TEST_FILE_PATH);
 
        printk("Suspending SPI3...\n");
        int ret = spim_nrfx_pm_action(
            spi3_dev,
            PM_DEVICE_ACTION_SUSPEND
        );
        if (ret != 0) {
            printk("Suspend failed: %d\n", ret);
            return;
        }
 
       
        printk("SPI3 is now suspended. Sleeping to allow deep sleep for 20s...\n");
 
        k_sleep(K_SECONDS(20));
 
        printk("Resuming SPI3...\n");
        ret = spim_nrfx_pm_action(
            spi3_dev,
            PM_DEVICE_ACTION_RESUME
        );
        if (ret != 0) {
            printk("Resume failed: %d\n", ret);
            return;
        }
 
        printk("SPI3 re-enabled.\n");
 
        printk("Cycle complete. Sleeping for 3s...\n");
        k_sleep(K_SECONDS(3));
       
    }
}
 
 
 
 
int main(void)
{
    int ret;
 
    setup_disk();
 
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
 ret = enable_usb_device_next();
#else
 ret = usb_enable(NULL);
#endif
 if (ret != 0) {
     LOG_ERR("Failed to enable USB");
     return 0;
 }
 
//  LOG_INF("The device is put in USB mass storage mode.\n");
    // benchmark_file_read(TEST_FILE_PATH);
    // benchmark_file_read_double_buffered(TEST_FILE_PATH);
 
    // spi3_power_cycle_test();
    return 0;
}