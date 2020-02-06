#include "NuMicro.h"
#include "region_defs.h"
#include "boot_record.h"
#include "tfm_boot_status.h"

#define MCUBOOT_IMAGE_NUMBER    1
#include "flash_map.h"



#if 0
    uint32_t ih_magic;
    uint32_t ih_load_addr;
    uint16_t ih_hdr_size;            /* Size of image header (bytes). */
    uint16_t ih_protect_tlv_size;    /* Size of protected TLV area (bytes). */
    uint32_t ih_img_size;            /* Does not include header. */
    uint32_t ih_flags;               /* IMAGE_F_[...]. */
    struct image_version ih_ver;
    uint32_t _pad1;
#endif

struct image_header g_imageHeader = {
    IMAGE_TLV_INFO_MAGIC,
    0x8000,
    IMAGE_HEADER_SIZE,
    0,
    0x2c800,
    IMAGE_F_RAM_LOAD,
    {0,0,0,0},
    0
};

#if 0

    /**
     * This flash area's ID; unique in the system.
     */
    uint8_t fa_id;

    /**
     * ID of the flash device this area is a part of.
     */
    uint8_t fa_device_id;

    uint16_t pad16;

    /**
     * This area's offset, relative to the beginning of its flash
     * device's storage.
     */
    uint32_t fa_off;

    /**
     * This area's size, in bytes.
     */
    uint32_t fa_size;

#endif

struct flash_area g_SPEArea = {
    0,
    0,
    0,
    0x8000,
    0x40000-0x8000,
};

struct flash_area g_NSPEArea = {
    0,
    0,
    0,
    0x10040000,
    0x40000,
};


int32_t UpdateBootStatus(void)
{
    enum boot_status_err_t rc;
    
    /* Save boot status to shared memory area */
    rc = boot_save_boot_status(SW_SPE,
                               &g_imageHeader,
                               &g_SPEArea
                              );
    if(rc)
    {
        printf("Failed to add SPE data to shared area");
        return -1;
    }
    
    rc = boot_save_boot_status(SW_NSPE,
                               &g_imageHeader,
                               &g_NSPEArea
                              );
    if(rc)
    {
        printf("Failed to add NSPE data to shared area");
        return -1;
    }
    
    
    return 0;
}


