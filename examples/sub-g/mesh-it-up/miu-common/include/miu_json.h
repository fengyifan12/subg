/*
 * miu_json.h — 轻量 JSON 字段提取工具（无动态内存分配）
 *
 * 只处理我们私有协议中出现的简单 flat 结构，不支持嵌套数组/递归解析。
 * 所有函数均为 static inline，include 即用，无需单独编译单元。
 */

#ifndef MIU_JSON_H
#define MIU_JSON_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * 在 json 字符串中查找 "key":"value"，把 value 写入 out。
 * 返回写入字节数，-1 表示未找到。
 */
static inline int miu_json_get_str(const char *json, const char *key,
                                   char *out, int max_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);

    const char *p = strstr(json, search);
    if (!p) return -1;

    p += strlen(search);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') return -1;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return -1;
    p++;

    int i = 0;
    while (*p && *p != '"' && i < max_len - 1) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return i;
}

/*
 * 在 json 字符串中查找 "key":number，把 number 写入 out。
 * 返回 0 成功，-1 未找到。
 */
static inline int miu_json_get_int(const char *json, const char *key, int *out)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);

    const char *p = strstr(json, search);
    if (!p) return -1;

    p += strlen(search);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') return -1;
    p++;
    while (*p == ' ' || *p == '\t') p++;

    if (*p != '-' && (*p < '0' || *p > '9')) return -1;
    *out = (int)strtol(p, NULL, 10);
    return 0;
}

#endif /* MIU_JSON_H */
