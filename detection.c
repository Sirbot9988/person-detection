/* 
CURRENT ISSUES: 
CNN is not constructing, lets try optimizing for L2 memory.
https://greenwaves-technologies.com/manuals_gap9/gap9_sdk_doc/html/source/tools/docs/autotiler.html#at-graph-control-code

*/

#include "stdio.h"
#include "pmsis.h"
#include "bsp/buffer.h"
#include "bsp/bsp.h"
#include "bsp/ai_deck.h"
#include "bsp/camera/himax.h"
#include "gaplib/jpeg_encoder.h"
#include "cpx.h"
#include "wifi.h"
#include "detection.h"
#include "detectionKernels.h"

// /* incl if code does not need L3 Flash . Otherwise detectionKernel will call L3 Flash leading to unknown refernece error. */
AT_HYPERFLASH_FS_EXT_ADDR_TYPE detection_L3_Flash = 0;
#define CAM_FULL_WIDTH   324
#define CAM_FULL_HEIGHT  244
#define CAM_WIDTH        324
#define CAM_HEIGHT       244

#define CHANNELS 1
#define IO RGB888_IO
#define CAT_LEN sizeof(uint32_t)

#define __XSTR(__s) __STR(__s)
#define __STR(__s) #__s


#define IMG_ORIENTATION 0x0101
uint8_t set_value2 = 1;
uint8_t set_value3 = 16;
uint8_t set_value4 = 1;
//define max objects to detect, as defined in training
#define MAX_OBJECTS      1

#ifndef STACK_SIZE
#define STACK_SIZE       (1024 * 6)
#endif

#ifndef SLAVE_STACK_SIZE
#define SLAVE_STACK_SIZE (1024)
#endif

#define LED_PIN          2
#define THRESHOLD        0.2f  // Confidence threshold
#define JPEG_BUFFER_SIZE (50 * 1024) // Adjust if needed

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

static int wifiConnected = 0;
static int wifiClientConnected = 0;

static pi_task_t task1;
static CPXPacket_t rxp;
static CPXPacket_t txp;

static unsigned char *cameraBufferFull;
static unsigned char *inputBuffer; // cameraBuffer normalized to [0..1]
static signed char *Output_1; // Q7 ? Re-examine 

static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;
static struct pi_device gpio_device;

// JPEG Encoder variables
static struct pi_device jpeg_encoder;
static jpeg_encoder_t encoder_struct;
static pi_buffer_t header;
static uint32_t headerSize;
static pi_buffer_t footer;
static uint32_t footerSize;
static pi_buffer_t jpeg_data;
static uint32_t jpegSize;
static pi_buffer_t buffer;

typedef struct
{
    uint8_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t depth;
    uint8_t type;
    uint32_t size;
} __attribute__((packed)) img_header_t;

typedef enum
{
    RAW_ENCODING = 0,
    JPEG_ENCODING = 1
} __attribute__((packed)) StreamerMode_t;

static StreamerMode_t streamerMode = JPEG_ENCODING;

// draw 255 (white) on the edges of the bounding box.
static void DrawRectangle(unsigned char *img, int img_w, int img_h,
                          int x, int y, int w, int h)
{
    /* This draws a white (255) rectangle boundary onto 'img' at coords (x,y,w,h). */
    int x2 = x + w;
    int y2 = y + h;

    if (x < 0)   x = 0;
    if (y < 0)   y = 0;
    if (x2 >= img_w)  x2 = img_w - 1;
    if (y2 >= img_h)  y2 = img_h - 1;

    // Top & bottom edges
    for (int X = x; X <= x2; X++)
    {
        if (y >= 0 && y < img_h)
            img[y * img_w + X] = 255;    // top edge
        if (y2 >= 0 && y2 < img_h)
            img[y2 * img_w + X] = 255;   // bottom edge
    }
    // Left & right edges
    for (int Y = y; Y <= y2; Y++)
    {
        if (x >= 0 && x < img_w)
            img[Y * img_w + x] = 255;    // left edge
        if (x2 >= 0 && x2 < img_w)
            img[Y * img_w + x2] = 255;   // right edge
    }
}

static void capture_done_cb(void *arg)
{
    cpxPrintToConsole(LOG_TO_CRTP, "Capture done callback triggered.\n");
    xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

static void rx_task(void *parameters)
{
    (void)parameters;
    while (1)
    {
        cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);
        WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t *)rxp.data;

        switch (wifiCtrl->cmd)
        {
        case WIFI_CTRL_STATUS_WIFI_CONNECTED:
            cpxPrintToConsole(LOG_TO_CRTP, "Wi-Fi connected (%u.%u.%u.%u)\n",
                              wifiCtrl->data[0], wifiCtrl->data[1],
                              wifiCtrl->data[2], wifiCtrl->data[3]);
            wifiConnected = 1;
            break;
        case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
            cpxPrintToConsole(LOG_TO_CRTP, "Wi-Fi client connection status: %u\n", wifiCtrl->data[0]);
            wifiClientConnected = wifiCtrl->data[0];
            break;
        default:
            cpxPrintToConsole(LOG_TO_CRTP, "Unknown Wi-Fi CTRL command: %u\n", wifiCtrl->cmd);
            break;
        }
    }
}

static void createImageHeaderPacket(CPXPacket_t *packet, uint32_t imgSize, StreamerMode_t imgType)
{
    img_header_t *imgHeader = (img_header_t *)packet->data;
    imgHeader->magic = 0xBC;
    imgHeader->width = CAM_FULL_WIDTH;
    imgHeader->height = CAM_FULL_HEIGHT;
    imgHeader->depth = 1;
    imgHeader->type = JPEG_ENCODING;
    imgHeader->size = imgSize;
    packet->dataLength = sizeof(img_header_t);
}

static void sendBufferViaCPX(CPXPacket_t *packet, uint8_t *buffer, uint32_t bufferSize)
{
    uint32_t offset = 0;
    while (offset < bufferSize)
    {
        uint32_t size = sizeof(packet->data);
        if (offset + size > bufferSize)
        {
            size = bufferSize - offset;
        }
        memcpy(packet->data, &buffer[offset], size);
        packet->dataLength = size;
        cpxSendPacketBlocking(packet);
        offset += size;
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

static int open_camera(struct pi_device *device)
{
  struct pi_himax_conf cam_conf;
  cpxPrintToConsole(LOG_TO_CRTP, "entered open camera\n");

  pi_himax_conf_init(&cam_conf);
  cpxPrintToConsole(LOG_TO_CRTP, "entered open camera\n");

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

static void RunNetwork()
{
    // Run the CNN inference.
    __PREFIX(CNN)(inputBuffer, Output_1);
}

static void cam_handler(void *arg) {
    (void)arg;
    cpxPrintToConsole(LOG_TO_CRTP, "Entering cam_handler\n");
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // Resize and normalize the input buffer to 61x81
    int new_width = 61;
    int new_height = 81;
    float scale_x = (float)CAM_FULL_WIDTH / new_width;
    float scale_y = (float)CAM_FULL_HEIGHT / new_height;

    for (int y = 0; y < new_height; y++)
    {
        for (int x = 0; x < new_width; x++)
        {
            int src_x = (int)(x * scale_x);
            int src_y = (int)(y * scale_y);
            inputBuffer[y * new_width + x] = (signed char)cameraBufferFull[src_y * CAM_FULL_WIDTH + src_x] - 128;
        }
    }

    cpxPrintToConsole(LOG_TO_CRTP, "sending inference\n");

    pi_cluster_send_task_to_cl(&cluster_dev, task);

    // Print Output_2 in Hex
    cpxPrintToConsole(LOG_TO_CRTP, "CNN Output_1 (Hex): ");
    cpxPrintToConsole(LOG_TO_CRTP, "%02X \n", (unsigned char)Output_1[0]);
    cpxPrintToConsole(LOG_TO_CRTP, "%02X \n", (unsigned char)Output_1[1]);
    cpxPrintToConsole(LOG_TO_CRTP, "%02X \n", (unsigned char)Output_1[2]);
    cpxPrintToConsole(LOG_TO_CRTP, "%02X \n", (unsigned char)Output_1[3]);
    cpxPrintToConsole(LOG_TO_CRTP, "First 10 bytes of normalized input buffer: ");
    for (int i = 0; i < 10; i++) {
        cpxPrintToConsole(LOG_TO_CRTP, "%02X ", inputBuffer[i]);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "\n");
    // int x_c  = (Output_2[offset + 0] + 128) / 2;
    // int y_c  = (Output_2[offset + 1] + 128) / 2;
    // int w_n  = (Output_2[offset + 2] + 128) / 2;
    // int h_n  = (Output_2[offset + 3] + 128) / 2;
    // int x_min = x_c - w_n / 2;
    // int x_max = x_c + w_n / 2;
    // int y_min = y_c - h_n / 2;
    // int y_max = y_c + h_n / 2;

    // // Scale to image resolution
    // int px = (x_min * CAM_FULL_WIDTH);
    // int py = (y_min * CAM_FULL_HEIGHT);
    // int pw = ((x_max - x_min) * CAM_FULL_WIDTH);
    // int ph = ((y_max - y_min) * CAM_FULL_HEIGHT);
    // Print raw values of Output_1 and Output_2
    for (int i = 0; i < 4; i++) {
        cpxPrintToConsole(LOG_TO_CRTP, "Raw Output_1[%d]: %d\n", i, (int)Output_1[i]);
    }
    float x_c = ((float)Output_1[0] + 128) / 256;  // Normalize if using Q7
    float y_c = ((float)Output_1[1] + 128) / 256;
    float w_n = ((float)Output_1[2] + 128) / 256;
    float h_n = ((float)Output_1[3] + 128) / 256;

    cpxPrintToConsole(LOG_TO_CRTP, "Bounding box normalized: x=%.2f, y=%.2f, w=%.2f, h=%.2f\n", x_c, y_c, w_n, h_n);
    // Convert to pixel coordinates
    int px = ((x_c - w_n / 2.0f) * CAM_FULL_WIDTH);
    int py = ((y_c - h_n / 2.0f) * CAM_FULL_HEIGHT);
    int pw = (w_n * CAM_FULL_WIDTH);
    int ph = (h_n * CAM_FULL_HEIGHT);

    cpxPrintToConsole(LOG_TO_CRTP, "Bounding box coordinates: px=%d, py=%d, pw=%d, ph=%d\n", px, py, pw, ph);
    
    int foundAny = 0;
    DrawRectangle(cameraBufferFull, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, px, py, pw, ph);

    if (!foundAny)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "No objects above threshold\n");
    }
    if (wifiClientConnected == 1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Encoding image as JPEG\n");

        uint32_t start_time = xTaskGetTickCount();
        jpeg_encoder_process(&encoder_struct, &buffer, &jpeg_data, &jpegSize);
        uint32_t encodingTime = xTaskGetTickCount() - start_time;

        uint32_t imgSize = headerSize + jpegSize + footerSize;

        // Send image header
        createImageHeaderPacket(&txp, imgSize, JPEG_ENCODING);
        cpxSendPacketBlocking(&txp);

        start_time = xTaskGetTickCount();
        // jpeg header send
        memcpy(txp.data, header.data, headerSize);
        txp.dataLength = headerSize;
        cpxSendPacketBlocking(&txp);

        // jpeg data send
        sendBufferViaCPX(&txp, (uint8_t*) jpeg_data.data, jpegSize);

        // jpeg footer conclude
        memcpy(txp.data, footer.data, footerSize);
        txp.dataLength = footerSize;
        cpxSendPacketBlocking(&txp);

        uint32_t transferTime = xTaskGetTickCount() - start_time;
        cpxPrintToConsole(LOG_TO_CRTP, "encoding=%d ms (%d bytes), transfer=%d ms\n",
                          encodingTime, imgSize, transferTime);
    }
    else
    {
        cpxPrintToConsole(LOG_TO_CRTP, "No Wi-Fi client connected, skipping image send\n");
        vTaskDelay(1000);

    }
    
    cpxPrintToConsole(LOG_TO_CRTP, "Starting next image capture\n");
    pi_camera_capture_async(&camera, cameraBufferFull, CAM_FULL_WIDTH * CAM_FULL_HEIGHT,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
}
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

static void camera_task(void *parameters)
{
    (void)parameters;
    vTaskDelay(10000); 

    setupWiFi();

    cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");
    /* Initialize Camera */
    if (open_camera(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return;
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Opened Camera\n");
    uint32_t resolution = CAM_FULL_WIDTH * CAM_FULL_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);
    /* Set Camera Buffers */
    cameraBufferFull = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (!cameraBufferFull)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate cameraBufferFull\n");
        return;
    }

    // cameraBufferResized = (unsigned char *)pmsis_l2_malloc(CAM_WIDTH * CAM_HEIGHT);
    // if (!cameraBufferResized)
    // {
    //     cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate cameraBufferResized\n");
    //     return;
    // }
    /* Set CNN Buffers*/
    inputBuffer = (unsigned char *)pmsis_l2_malloc(CAM_WIDTH * CAM_HEIGHT * sizeof(unsigned char));
    if (inputBuffer == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate inputBuffer\n");
        return;
    }
    Output_1 = (signed char *)pmsis_l2_malloc(4 * MAX_OBJECTS * sizeof(signed char));
    if (!Output_1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Output_1\n");
        return;
    }


    /* Configure CNN task */
    pi_cluster_conf_init(&cluster_conf);
    pi_open_from_conf(&cluster_dev, (void *)&cluster_conf);
    pi_cluster_open(&cluster_dev);
    task = pmsis_l2_malloc(sizeof(struct pi_cluster_task));
    if (!task)
    {  
        cpxPrintToConsole(LOG_TO_CRTP, "failed to allocate memory for task\n");
    }
    cpxPrintToConsole(LOG_TO_CRTP,"Allocated memory for task\n");

    memset(task, 0, sizeof(struct pi_cluster_task));
    task->entry = &RunNetwork;
    task->stack_size = STACK_SIZE;             // defined in makefile
    task->slave_stack_size = SLAVE_STACK_SIZE; // "
    task->arg = NULL;

    int ret = __PREFIX(CNN_Construct)();
    if (ret)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to construct CNN with %d\n", ret);
        pmsis_exit(-5);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Constructed CNN\n");

    /*Everything related to JPEG Encoder*/
    struct jpeg_encoder_conf enc_conf;
    jpeg_encoder_conf_init(&enc_conf);
    enc_conf.width = CAM_FULL_WIDTH;
    enc_conf.height = CAM_FULL_HEIGHT;
    enc_conf.flags = 0;

    if (jpeg_encoder_open(&encoder_struct, &enc_conf))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to initialize JPEG encoder\n");
        return;
    } else { 
        cpxPrintToConsole(LOG_TO_CRTP, "Initialized JPEG encoder\n");
    }

    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, cameraBufferFull);
    cpxPrintToConsole(LOG_TO_CRTP, "Initialized buffer\n");
    pi_buffer_set_format(&buffer, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
    cpxPrintToConsole(LOG_TO_CRTP, "Set buffer format\n");
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
    } else { 
        cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for JPEG structures\n");
    }
    
    jpeg_encoder_header(&encoder_struct, &header, &headerSize);
    cpxPrintToConsole(LOG_TO_CRTP, "Encoded header\n");
    jpeg_encoder_footer(&encoder_struct, &footer, &footerSize);
    cpxPrintToConsole(LOG_TO_CRTP, "Encoded footer\n");
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    cpxPrintToConsole(LOG_TO_CRTP, "Stopped camera\n");
    // Initialize route once
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);
    cpxPrintToConsole(LOG_TO_CRTP, "Initialized route\n");
    
    /* Stop Camera and init settings for it to work */

    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    pi_camera_reg_set(&camera, 0x2100, &set_value2); // AE_CTRL
    pi_camera_reg_set(&camera, 0x0205, &set_value3); // ANALOG_GLOBAL_GAIN: 0x10 = 2x, 0x20 = 4x
    pi_camera_reg_set(&camera, 0x0104, &set_value4); // This is needed for the camera to actually update its registers.

    pi_camera_capture_async(&camera, cameraBufferFull, resolution,
                            pi_task_callback(&task1, cam_handler, NULL));
    cpxPrintToConsole(LOG_TO_CRTP, "Started first image capture\n");
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    cpxPrintToConsole(LOG_TO_CRTP, "Started camera\n");

    while (1)
    {
        pi_yield();
    }

    /* Destruct CNN */
    __PREFIX(CNN_Destruct)();
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
        printf("[UART] open failed!\n");
        pmsis_exit(-1);
    }

    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);
    cpxPrintToConsole(LOG_TO_CRTP, "-- PERSON DETECTION --\n");

    evGroup = xEventGroupCreate();

    // Heartbeat task
    xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    // RX task
    xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Camera task
    if (xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 22,
                    NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "camera_task did not start!\n");
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
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    return pmsis_kickoff((void *)start_example);
}