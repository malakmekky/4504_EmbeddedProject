#pragma once
#include <stddef.h>
typedef int nvs_handle_t; typedef int esp_err_t;
#define ESP_OK 0
#define NVS_READWRITE 1
#define NVS_READONLY 0
static inline esp_err_t nvs_open(const char*n,int m,nvs_handle_t*h){return 1;}
static inline esp_err_t nvs_set_str(nvs_handle_t h,const char*k,const char*v){return 1;}
static inline esp_err_t nvs_get_str(nvs_handle_t h,const char*k,char*v,size_t*l){return 1;}
static inline esp_err_t nvs_commit(nvs_handle_t h){return 1;}
static inline void nvs_close(nvs_handle_t h){}
