/*
 * ble_link.h - BLE link to the PC app (NimBLE GATT server).
 *
 * Service   7a1e0001-3c2b-4d8e-9f10-2b6c5d4e3f21
 *   CMD     7a1e0002-...  write   : "TRAIN", "SOLVE", "STOP", "PATH", "STATUS", "FORGET"
 *   STATUS  7a1e0003-...  notify  : text lines ending in '\n' (long lines are split
 *                                   over several notifications, join until '\n')
 */
#pragma once

#include <stdbool.h>

#define LINK_MSG_LEN 160

typedef enum {
  CMD_TRAIN,
  CMD_SOLVE,
  CMD_PATH,
  CMD_STATUS,
  CMD_FORGET,
} robot_cmd_t;

// start BLE advertising + the sender task (call once, after nvs_flash_init)
void ble_link_init(void);

// next command from the PC, non-blocking
bool link_get_command(robot_cmd_t *cmd);

// STOP is a flag instead of a queued command so it can interrupt turns right away
bool link_stop_requested(void);
void link_clear_stop(void);

// printf-style status line: printed on serial and sent to the PC if connected
void link_send(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// same, but waits for room in the send queue (for long dumps like the map, never while driving)
void link_send_wait(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
