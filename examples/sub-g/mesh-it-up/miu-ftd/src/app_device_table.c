/*
 * app_device_table.c — Leader 设备表实现
 */

#include "app_device_table.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include <openthread/ip6.h>

static miu_device_table_t s_dev_table;

/* -----------------------------------------------------------------------
 * 初始化
 * ----------------------------------------------------------------------- */
void app_device_table_init(void)
{
    memset(&s_dev_table, 0, sizeof(s_dev_table));
    log_info("[devtab] init, capacity=%d", MIU_MAX_DEVICES);
}

/* -----------------------------------------------------------------------
 * 查找
 * ----------------------------------------------------------------------- */
miu_device_info_t *app_device_table_find(const char *dev_name)
{
    if (!dev_name) return NULL;
    for (int i = 0; i < MIU_MAX_DEVICES; i++) {
        if (s_dev_table.devices[i].valid &&
            strncmp(s_dev_table.devices[i].dev_name, dev_name,
                    MIU_DEV_NAME_MAX - 1) == 0) {
            return &s_dev_table.devices[i];
        }
    }
    return NULL;
}

/* -----------------------------------------------------------------------
 * 新增 / 更新
 * ----------------------------------------------------------------------- */
miu_device_info_t *app_device_table_add(const char        *dev_name,
                                         miu_dev_type_t     type,
                                         const otIp6Address *ip,
                                         uint16_t            rloc16,
                                         const char         *fw_ver,
                                         const char         *hw_ver)
{
    if (!dev_name || !ip) return NULL;

    /* 已存在则复用，否则找空槽 */
    miu_device_info_t *entry = app_device_table_find(dev_name);
    if (!entry) {
        for (int i = 0; i < MIU_MAX_DEVICES; i++) {
            if (!s_dev_table.devices[i].valid) {
                entry = &s_dev_table.devices[i];
                s_dev_table.count++;
                break;
            }
        }
    }
    if (!entry) {
        log_info("[devtab] table full, drop %s", dev_name);
        return NULL;
    }

    taskENTER_CRITICAL();
    entry->valid        = true;
    entry->dev_type     = type;
    entry->rloc16       = rloc16;
    entry->ip           = *ip;
    entry->last_seen_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    strncpy(entry->dev_name, dev_name, MIU_DEV_NAME_MAX - 1);
    entry->dev_name[MIU_DEV_NAME_MAX - 1] = '\0';
    if (fw_ver) {
        strncpy(entry->fw_ver, fw_ver, MIU_FW_VER_MAX - 1);
        entry->fw_ver[MIU_FW_VER_MAX - 1] = '\0';
    }
    if (hw_ver) {
        strncpy(entry->hw_ver, hw_ver, MIU_HW_VER_MAX - 1);
        entry->hw_ver[MIU_HW_VER_MAX - 1] = '\0';
    }
    taskEXIT_CRITICAL();

    char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(ip, ip_str, sizeof(ip_str));
    log_info("[devtab] +%s  type=%-8s rloc=%04X  ip=%s  fw=%s",
             dev_name, app_device_type_to_str(type), rloc16, ip_str, entry->fw_ver);
    return entry;
}

/* -----------------------------------------------------------------------
 * 路由查询
 * ----------------------------------------------------------------------- */
bool app_device_table_get_ip(const char *dev_name, otIp6Address *out_ip)
{
    miu_device_info_t *entry = app_device_table_find(dev_name);
    if (!entry || !out_ip) return false;
    *out_ip = entry->ip;
    return true;
}

/* -----------------------------------------------------------------------
 * 调试打印
 * ----------------------------------------------------------------------- */
void app_device_table_dump(void)
{
    log_info("[devtab] ======= device table (%u / %u) =======",
             s_dev_table.count, MIU_MAX_DEVICES);
    for (int i = 0; i < MIU_MAX_DEVICES; i++) {
        miu_device_info_t *d = &s_dev_table.devices[i];
        if (!d->valid) continue;
        char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&d->ip, ip_str, sizeof(ip_str));
        log_info("  [%2d] %-14s  type=%-8s  rloc=%04X  fw=%-8s  ip=%s",
                 i, d->dev_name, app_device_type_to_str(d->dev_type),
                 d->rloc16, d->fw_ver, ip_str);
    }
    log_info("[devtab] ==========================================");
}

/* -----------------------------------------------------------------------
 * 类型字符串互转
 * ----------------------------------------------------------------------- */
miu_dev_type_t app_device_type_from_str(const char *str)
{
    if (!str)                        return MIU_DEV_TYPE_UNKNOWN;
    if (strcmp(str, "SOCKET") == 0)  return MIU_DEV_TYPE_SOCKET;
    if (strcmp(str, "LIGHT")  == 0)  return MIU_DEV_TYPE_LIGHT;
    if (strcmp(str, "RADAR")  == 0)  return MIU_DEV_TYPE_RADAR;
    if (strcmp(str, "RGBCW")  == 0)  return MIU_DEV_TYPE_RGBCW;
    return MIU_DEV_TYPE_UNKNOWN;
}

const char *app_device_type_to_str(miu_dev_type_t type)
{
    switch (type) {
    case MIU_DEV_TYPE_SOCKET:  return "SOCKET";
    case MIU_DEV_TYPE_LIGHT:   return "LIGHT";
    case MIU_DEV_TYPE_RADAR:   return "RADAR";
    case MIU_DEV_TYPE_RGBCW:   return "RGBCW";
    default:                    return "UNKNOWN";
    }
}
