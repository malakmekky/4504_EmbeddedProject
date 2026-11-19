/*
 * ble_link.c - NimBLE GATT server.
 *
 * Tasks (both on core 0, the robot runs alone on core 1):
 *   nimble host task : BLE stack, calls gatt_access() when the PC writes a command
 *   ble_tx task      : takes status lines from txQueue and sends them as notifications
 *
 * Commands go to the robot through cmdQueue, status lines come back through txQueue,
 * so the robot task never waits on the radio.
 */
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "config.h"
#include "ble_link.h"

// UUIDs are written least significant byte first
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
  0x21, 0x3f, 0x4e, 0x5d, 0x6c, 0x2b, 0x10, 0x9f, 0x8e, 0x4d, 0x2b, 0x3c, 0x01, 0x00, 0x1e, 0x7a);
static const ble_uuid128_t CMD_UUID = BLE_UUID128_INIT(
  0x21, 0x3f, 0x4e, 0x5d, 0x6c, 0x2b, 0x10, 0x9f, 0x8e, 0x4d, 0x2b, 0x3c, 0x02, 0x00, 0x1e, 0x7a);
static const ble_uuid128_t STATUS_UUID = BLE_UUID128_INIT(
  0x21, 0x3f, 0x4e, 0x5d, 0x6c, 0x2b, 0x10, 0x9f, 0x8e, 0x4d, 0x2b, 0x3c, 0x03, 0x00, 0x1e, 0x7a);

static QueueHandle_t cmdQueue;
static QueueHandle_t txQueue;

static volatile bool stopFlag = false;
static volatile bool connected = false;
static volatile bool notifyOn = false;

static uint16_t connHandle;
static uint16_t statusHandle;
static uint8_t ownAddrType;

static char lastMsg[LINK_MSG_LEN] = "READY";

static void advertise(void);

// ---------- commands from the PC ----------

static void handleCommandText(char *s) {

  // trim + uppercase
  while (*s && isspace((unsigned char)*s)) s++;
  char *end = s + strlen(s);
  while (end > s && isspace((unsigned char)end[-1])) *--end = '\0';
  for (char *p = s; *p; p++) *p = toupper((unsigned char)*p);

  if (strcmp(s, "STOP") == 0) {
    stopFlag = true;
    return;
  }

  robot_cmd_t cmd;

  if (strcmp(s, "TRAIN") == 0)       cmd = CMD_TRAIN;
  else if (strcmp(s, "SOLVE") == 0)  cmd = CMD_SOLVE;
  else if (strcmp(s, "PATH") == 0)   cmd = CMD_PATH;
  else if (strcmp(s, "STATUS") == 0) cmd = CMD_STATUS;
  else if (strcmp(s, "FORGET") == 0) cmd = CMD_FORGET;
  else {
    link_send("ERR unknown command '%s'", s);
    return;
  }

  xQueueSend(cmdQueue, &cmd, 0);
}

static int gatt_access(uint16_t conn, uint16_t attr,
                       struct ble_gatt_access_ctxt *ctxt, void *arg) {

  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {

    char buf[32];
    uint16_t len = 0;

    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    buf[len] = '\0';
    handleCommandText(buf);
    return 0;
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    int rc = os_mbuf_append(ctxt->om, lastMsg, strlen(lastMsg));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gattServices[] = {
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &SVC_UUID.u,
    .characteristics = (struct ble_gatt_chr_def[]) {
      {
        .uuid = &CMD_UUID.u,
        .access_cb = gatt_access,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
      },
      {
        .uuid = &STATUS_UUID.u,
        .access_cb = gatt_access,
        .val_handle = &statusHandle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
      },
      { 0 }
    },
  },
  { 0 }
};

// ---------- connection handling ----------

static int gap_event(struct ble_gap_event *event, void *arg) {

  switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        connHandle = event->connect.conn_handle;
        connected = true;
        printf("BLE connected\n");
      } else {
        advertise();
      }
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      connected = false;
      notifyOn = false;
      printf("BLE disconnected\n");
      advertise();
      break;

    case BLE_GAP_EVENT_SUBSCRIBE:
      if (event->subscribe.attr_handle == statusHandle) {
        notifyOn = event->subscribe.cur_notify;
        if (notifyOn) {
          // PC just started listening, tell it where we are
          robot_cmd_t cmd = CMD_STATUS;
          xQueueSend(cmdQueue, &cmd, 0);
        }
      }
      break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      advertise();
      break;

    default:
      break;
  }

  return 0;
}

static void advertise(void) {

  struct ble_hs_adv_fields fields = { 0 };
  const char *name = ble_svc_gap_device_name();

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;
  ble_gap_adv_set_fields(&fields);

  struct ble_gap_adv_params params = { 0 };
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  ble_gap_adv_start(ownAddrType, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
}

static void on_sync(void) {
  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &ownAddrType);
  advertise();
  printf("BLE advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

static void on_reset(int reason) {
  printf("BLE reset, reason %d\n", reason);
}

static void host_task(void *arg) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

// ---------- sending status lines ----------

static void tx_task(void *arg) {

  char msg[LINK_MSG_LEN + 1];

  while (1) {

    if (xQueueReceive(txQueue, msg, portMAX_DELAY) != pdTRUE)
      continue;

    if (!connected || !notifyOn)
      continue;

    size_t len = strnlen(msg, LINK_MSG_LEN - 1);
    msg[len++] = '\n';

    // a notification carries at most MTU - 3 bytes
    uint16_t mtu = ble_att_mtu(connHandle);
    size_t chunk = mtu > 23 ? mtu - 3 : 20;

    size_t off = 0;
    int tries = 0;

    while (off < len && connected && tries < 20) {

      size_t n = (len - off < chunk) ? len - off : chunk;
      struct os_mbuf *om = ble_hs_mbuf_from_flat(msg + off, n);

      if (om != NULL && ble_gatts_notify_custom(connHandle, statusHandle, om) == 0) {
        off += n;
        tries = 0;
      } else {
        tries++;                       // stack buffers full, wait a bit and retry
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
  }
}

// ---------- public ----------

static void vsend(TickType_t wait, const char *fmt, va_list args) {

  char buf[LINK_MSG_LEN];
  vsnprintf(buf, sizeof(buf), fmt, args);

  printf("%s\n", buf);
  strncpy(lastMsg, buf, sizeof(lastMsg) - 1);

  if (txQueue != NULL && connected && notifyOn)
    xQueueSend(txQueue, buf, wait);
}

void link_send(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsend(0, fmt, args);                 // never block the robot, drop if full
  va_end(args);
}

void link_send_wait(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsend(pdMS_TO_TICKS(500), fmt, args);
  va_end(args);
}

bool link_get_command(robot_cmd_t *cmd) {
  return cmdQueue != NULL && xQueueReceive(cmdQueue, cmd, 0) == pdTRUE;
}

bool link_stop_requested(void) {
  return stopFlag;
}

void link_clear_stop(void) {
  stopFlag = false;
}

void ble_link_init(void) {

  cmdQueue = xQueueCreate(8, sizeof(robot_cmd_t));
  txQueue = xQueueCreate(24, LINK_MSG_LEN);

  ESP_ERROR_CHECK(nimble_port_init());

  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.reset_cb = on_reset;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  ESP_ERROR_CHECK(ble_gatts_count_cfg(gattServices));
  ESP_ERROR_CHECK(ble_gatts_add_svcs(gattServices));

  ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

  xTaskCreatePinnedToCore(tx_task, "ble_tx", 4096, NULL, 4, NULL, 0);
  nimble_port_freertos_init(host_task);
}
