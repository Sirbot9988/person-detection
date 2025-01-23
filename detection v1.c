/* 
 * Adding patch model, with sliding window. 
 * 
 */
#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/ai_deck.h"
#include "bsp/camera/himax.h"
#include "gaplib/jpeg_encoder.h"
#include "cpx.h"
#include "wifi.h"
#include "detection.h"
#include "detectionKernels.h"

AT_HYPERFLASH_FS_EXT_ADDR_TYPE detection_L3_Flash = 0;

// Camera configuration
#define CAM_FULL_WIDTH   324
#define CAM_FULL_HEIGHT  244
#define IMG_ORIENTATION  0x0101

// Detection parameters
#define PATCH_SIZE       64
#define STRIDE           32  
#define THRESHOLD        0.5f

// System configuration
#define JPEG_BUFFER_SIZE (80 * 1024)
#define LED_PIN          2

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

static pi_task_t task1;
static CPXPacket_t rxp, txp;

// Camera resources
static unsigned char *cameraBufferFull;
static signed char *patchBuffer;
static signed char *Output_1;

static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;
static struct pi_device gpio_device;


// JPEG encoder
static struct pi_device jpeg_encoder;
static jpeg_encoder_t encoder_struct;
static pi_buffer_t header;
static uint32_t headerSize;
static pi_buffer_t footer;
static uint32_t footerSize;
static pi_buffer_t jpeg_data;
static uint32_t jpegSize;
static pi_buffer_t buffer;


// WiFi status
static int wifiConnected = 0;
static int wifiClientConnected = 0;

typedef struct {
    uint8_t  magic;
    uint16_t width;
    uint16_t height;
    uint8_t  depth;
    uint8_t  type;
    uint32_t size;
} __attribute__((packed)) img_header_t;

typedef enum {
    RAW_ENCODING  = 0,
    JPEG_ENCODING = 1
} StreamerMode_t;

static StreamerMode_t streamerMode = JPEG_ENCODING;

// Detection functions
static void DrawRectangle(unsigned char *img, int img_w, int img_h,
                          int x, int y, int w, int h) {
    int x2 = x + w - 1;
    int y2 = y + h - 1;

    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x2 >= img_w) x2 = img_w - 1;
    if (y2 >= img_h) y2 = img_h - 1;

    for (int X = x; X <= x2; X++) {
        img[y * img_w + X]  = 255;
        img[y2 * img_w + X] = 255;
    }
    for (int Y = y; Y <= y2; Y++) {
        img[Y * img_w + x]  = 255;
        img[Y * img_w + x2] = 255;
    }
}

static void RunNetwork() {
    __PREFIX(CNN)(patchBuffer, Output_1);
}

static void processSlidingWindow(uint8_t *img) {
    int foundAny = 0;

    for (int y = 0; y <= CAM_FULL_HEIGHT - PATCH_SIZE; y += STRIDE) {
        for (int x = 0; x <= CAM_FULL_WIDTH - PATCH_SIZE; x += STRIDE) {
            for (int j = 0; j < PATCH_SIZE; j++) {
                for (int i = 0; i < PATCH_SIZE; i++) {
                    uint8_t pix = img[(y + j) * CAM_FULL_WIDTH + (x + i)];
                    float val = pix / 255.0f;
                    int scaled = (int)((val * 128.0f) - 128.0f);
                    patchBuffer[j * PATCH_SIZE + i] = (signed char)(scaled);
                }
            }

            RunNetwork();
            float confidence = ((float)Output_1[0] / 128.0f + 1.0f) / 2.0f;
            if (confidence >= THRESHOLD) {
                foundAny = 1;
                DrawRectangle(img, CAM_FULL_WIDTH, CAM_FULL_HEIGHT,
                              x, y, PATCH_SIZE, PATCH_SIZE);
            }
        }
    }
    if (!foundAny) {
        cpxPrintToConsole(LOG_TO_CRTP, "No detections\n");
    }
}

// Camera callback
static void capture_done_cb(void *arg) {
    xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

// WiFi handling
void rx_task(void *parameters) {
    while (1) {
        cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);
        WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t *)rxp.data;
        
        switch (wifiCtrl->cmd) {
            case WIFI_CTRL_STATUS_WIFI_CONNECTED:
                wifiConnected = 1;
                break;
            case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
                wifiClientConnected = wifiCtrl->data[0];
                break;
            default: break;
        }
    }
}

static CPXPacket_t txp;
// Image transmission
void createImageHeaderPacket(CPXPacket_t * packet, uint32_t imgSize, StreamerMode_t imgType) {
  img_header_t *imgHeader = (img_header_t *) packet->data;
  imgHeader->magic = 0xBC;
  imgHeader->width = CAM_FULL_WIDTH;
  imgHeader->height = CAM_FULL_HEIGHT;
  imgHeader->depth = 1;
  imgHeader->type = imgType;
  imgHeader->size = imgSize;
  packet->dataLength = sizeof(img_header_t);
}

void sendBufferViaCPX(CPXPacket_t *packet, uint8_t *buf, uint32_t size) {
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = sizeof(packet->data);
        if (offset + chunk > size) chunk = size - offset;
        memcpy(packet->data, &buf[offset], chunk);
        packet->dataLength = chunk;
        cpxSendPacketBlocking(packet);
        offset += chunk;
    }
}

static int open_camera(struct pi_device *device)
{
  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QVGA;

  pi_open_from_conf(device, &cam_conf);
  if (pi_camera_open(device))
    return -1;

  // rotate image
  pi_camera_control(device, PI_CAMERA_CMD_START, 0);
  uint8_t set_value = 3;
  uint8_t reg_value;
  pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
  pi_time_wait_us(1000000);
  pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);
  if (set_value != reg_value)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
    return -1;
  }
  pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
  pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

  return 0;
}

// Main camera task

static void cam_handler(void *arg)
{
    int foundAny = 0;

    for (int y = 0; y <= CAM_FULL_HEIGHT - PATCH_SIZE; y += STRIDE) {
        for (int x = 0; x <= CAM_FULL_WIDTH - PATCH_SIZE; x += STRIDE) {
            for (int j = 0; j < PATCH_SIZE; j++) {
                for (int i = 0; i < PATCH_SIZE; i++) {
                    uint8_t pix = cameraBufferFull[(y + j) * CAM_FULL_WIDTH + (x + i)];
                    float val = pix / 255.0f;
                    int scaled = (int)((val * 128.0f) - 128.0f);
                    patchBuffer[j * PATCH_SIZE + i] = (signed char)(scaled);
                    pi_cluster_send_task_to_cl(&cluster_dev, task);

                }
            };
            float confidence = ((float)Output_1[0] / 128.0f + 1.0f) / 2.0f;
            if (confidence >= THRESHOLD) {
                foundAny = 1;
                DrawRectangle(cameraBufferFull, CAM_FULL_WIDTH, CAM_FULL_HEIGHT,
                              x, y, PATCH_SIZE, PATCH_SIZE);
            }
        }
    }
    if (!foundAny) {
        cpxPrintToConsole(LOG_TO_CRTP, "No detections\n");
    }
}
static void setupWiFi(void)
{
    static char ssid[] = "GAP8-WiFi";
    cpxPrintToConsole(LOG_TO_CRTP, "Setting up Wi-Fi AP\n");

    txp.route.destination = CPX_T_ESP32;
    rxp.route.source = CPX_T_GAP8;
    txp.route.function = CPX_F_WIFI_CTRL;
    txp.route.version = CPX_VERSION;

    WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t *)txp.data;
    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    txp.dataLength = sizeof(ssid);
    cpxSendPacketBlocking(&txp);

    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01;
    txp.dataLength = 2;
    cpxSendPacketBlocking(&txp);
}

static void camera_task(void *parameters)
{
    (void)parameters;
    vTaskDelay(2000); 

    setupWiFi();

    cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");

    uint32_t resolution = CAM_FULL_WIDTH * CAM_FULL_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);

    cameraBufferFull = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (!cameraBufferFull)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate cameraBufferFull\n");
        return;
    }
    patchBuffer = (signed char *)pmsis_l2_malloc(PATCH_SIZE * PATCH_SIZE);
    if (!patchBuffer)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate patchBuffer\n");
        return;
    }

    Output_1 = (signed char *)pmsis_l2_malloc(sizeof(signed char));
    if (!Output_1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Output_1\n");
        return;
    }

    if (open_camera(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return;
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Opened Camera\n");

    // Initialize cluster
    pi_cluster_conf_init(&cluster_conf);
    pi_open_from_conf(&cluster_dev, &cluster_conf);
    pi_cluster_open(&cluster_dev);

    task = (struct pi_cluster_task *)pmsis_l2_malloc(sizeof(struct pi_cluster_task));
    if (!task)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate cluster task\n");
        return;
    }
    memset(task, 0, sizeof(struct pi_cluster_task));
    task->entry = &RunNetwork;
    task->stack_size = STACK_SIZE;
    task->slave_stack_size = SLAVE_STACK_SIZE;

    int ret = __PREFIX(CNN_Construct)();
    if (ret)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to construct CNN with %d\n", ret);
        pmsis_exit(-5);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Constructed CNN\n");

    // Initialize JPEG encoder
    struct jpeg_encoder_conf enc_conf;
    jpeg_encoder_conf_init(&enc_conf);
    enc_conf.width = CAM_FULL_WIDTH;
    enc_conf.height = CAM_FULL_HEIGHT;
    enc_conf.flags = 0;

    if (jpeg_encoder_open(&encoder_struct, &enc_conf))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to initialize JPEG encoder\n");
        return;
    }

    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, cameraBufferFull);
    pi_buffer_set_format(&buffer, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    header.size = 1024;
    header.data = pmsis_l2_malloc(1024);
    footer.size = 10;
    footer.data = pmsis_l2_malloc(10);
    jpeg_data.size = 1024 * 15;
    jpeg_data.data = pmsis_l2_malloc(1024 * 15);

    if (header.data == 0 || footer.data == 0 || jpeg_data.data == 0)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for JPEG structures\n");
        return;
    }

    jpeg_encoder_header(&encoder_struct, &header, &headerSize);
    jpeg_encoder_footer(&encoder_struct, &footer, &footerSize);

    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // Initialize route once
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);

    // Start first image capture
    pi_camera_capture_async(&camera, cameraBufferFull, resolution,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

    while (1)
    {
        pi_yield();
    }

    /* Destruct CNN */
    __PREFIX(CNN_Destruct)();
}

// heartbeat task. Blinks the LED on the board.
static void hb_task(void *parameters)
{
    (void)parameters;
    pi_gpio_pin_configure(&gpio_device, LED_PIN, PI_GPIO_OUTPUT);
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    while (1)
    {
        pi_gpio_pin_write(&gpio_device, LED_PIN, 1);
        vTaskDelay(xDelay);
        pi_gpio_pin_write(&gpio_device, LED_PIN, 0);
        vTaskDelay(xDelay);
    }
}

void start_example(void)
{
  struct pi_uart_conf conf;
  struct pi_device device;
  pi_uart_conf_init(&conf);
  conf.baudrate_bps = 115200;

  pi_open_from_conf(&device, &conf);
  if (pi_uart_open(&device))
  {
    printf("[UART] open failed !\n");
    pmsis_exit(-1);
  }

  cpxInit();
  cpxEnableFunction(CPX_F_WIFI_CTRL);

  cpxPrintToConsole(LOG_TO_CRTP, "-- Person Detection Patch Inference (Sangwoo) --\n");

  evGroup = xEventGroupCreate();

  BaseType_t xTask;

  xTask = xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "HB task did not start !\n");
    pmsis_exit(-1);
  }

  xTask = xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 4,
                      NULL, tskIDLE_PRIORITY + 1, NULL);

  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
    pmsis_exit(-1);
  }

  xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);

  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "RX task did not start !\n");
    pmsis_exit(-1);
  }

  while (1)
  {
    pi_yield();
  }
}

int main(void)
{
  pi_bsp_init();

  // Increase the FC freq to 250 MHz
  pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
  __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

  return pmsis_kickoff((void *)start_example);
}
