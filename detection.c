/**
 * @file detection_wifi_streaming.c
 *
 * AI-deck GAP8 - Neural Network Inference with Wi-Fi Streaming
 *
 * This program captures images from the camera, runs a neural network inference
 * on the captured images, and streams the images over Wi-Fi.
 *
 * This code is a merged version of the neural network inference example and
 * the Wi-Fi image streaming example.
 */

#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "gaplib/jpeg_encoder.h"
#include "stdio.h"

#include "cpx.h"
#include "wifi.h"
#include "detectionKernels.h" // Neural network kernels
#include "gaplib/ImgIO.h"

#include "bsp/ai_deck.h"
#include "bsp/transport/nina_w10.h"

#define IMG_ORIENTATION 0x0101

// Camera dimensions
#define CAM_FULL_WIDTH 324
#define CAM_FULL_HEIGHT 244

// Neural network input dimensions
#define CAM_WIDTH 80
#define CAM_HEIGHT 48

#define CHANNELS 1
#define IO RGB888_IO

#define STACK_SIZE  1024
#define SLAVE_STACK_SIZE 1024

#define __XSTR(__s) __STR(__s)
#define __STR(__s) #__s

static pi_task_t task1;
static unsigned char *cameraBufferFull;
static unsigned char *cameraBufferResized;
static signed short *Output_1; // Confidence score
static signed char *Output_2;  // Bounding Box

static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;

AT_HYPERFLASH_FS_EXT_ADDR_TYPE __PREFIX(_L3_Flash) = 0;

static int wifiConnected = 0;
static int wifiClientConnected = 0;

static CPXPacket_t rxp;
static CPXPacket_t txp;

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

// JPEG encoder variables
static jpeg_encoder_t jpeg_encoder;
pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;
pi_buffer_t jpeg_data;
uint32_t jpegSize;

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

#define IMG_HEADER_SIZE sizeof(img_header_t)

#define LED_PIN 2
static pi_device_t led_gpio_dev;

// Define this macro to enable Wi-Fi AP setup
#define SETUP_WIFI_AP

#ifdef SETUP_WIFI_AP
void setupWiFi(void)
{
    static char ssid[] = "WiFi streaming example";
    cpxPrintToConsole(LOG_TO_CRTP, "Setting up Wi-Fi AP\n");
    // Set up the routing for the Wi-Fi CTRL packets
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
#endif

static void resize_image(unsigned char *src, unsigned char *dst, int src_w, int src_h, int dst_w, int dst_h)
{
    int x_ratio = (int)((src_w << 16) / dst_w) + 1;
    int y_ratio = (int)((src_h << 16) / dst_h) + 1;
    for (int y = 0; y < dst_h; y++)
    {
        for (int x = 0; x < dst_w; x++)
        {
            int src_x = (x * x_ratio) >> 16;
            int src_y = (y * y_ratio) >> 16;
            dst[y * dst_w + x] = src[src_y * src_w + src_x];
        }
    }
}

static void RunNetwork()
{
    __PREFIX(CNN)(cameraBufferResized, Output_1, Output_2);
}

static void createImageHeaderPacket(CPXPacket_t *packet, uint32_t imgSize, StreamerMode_t imgType)
{
    img_header_t *imgHeader = (img_header_t *)packet->data;
    imgHeader->magic = 0xBC;
    imgHeader->width = CAM_FULL_WIDTH;
    imgHeader->height = CAM_FULL_HEIGHT;
    imgHeader->depth = 1;
    imgHeader->type = imgType;
    imgHeader->size = imgSize;
    packet->dataLength = IMG_HEADER_SIZE;
}

static void sendBufferViaCPX(CPXPacket_t *packet, uint8_t *buffer, uint32_t bufferSize)
{
    uint32_t offset = 0;
    uint32_t size = 0;
    do
    {
        size = sizeof(packet->data);
        if (offset + size > bufferSize)
        {
            size = bufferSize - offset;
        }
        memcpy(packet->data, &buffer[offset], size);
        packet->dataLength = size;
        cpxSendPacketBlocking(packet);
        offset += size;
    } while (offset < bufferSize);
}

static void cam_handler(void *arg)
{
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // Resize the captured image for neural network input
    resize_image(cameraBufferFull, cameraBufferResized, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, CAM_WIDTH, CAM_HEIGHT);

    // Run inference
    pi_cluster_send_task_to_cl(&cluster_dev, task);

    // Process neural network outputs
    float confidence = (float)Output_1[0] / 32768.0f;  // Map to [-1, 1]
    confidence = (confidence + 1.0f) / 2.0f;           // Map to [0, 1]

    if (confidence >= 0.5f)
    {
        // Object detected
        float bbox[4];
        for (int i = 0; i < 4; i++)
        {
            float bbox_value = (float)Output_2[i] / 128.0f; // Map to [-1, 1]
            bbox_value = (bbox_value + 1.0f) / 2.0f;        // Map to [0, 1]
            bbox[i] = bbox_value;
        }
        cpxPrintToConsole(LOG_TO_CRTP, "Object detected with confidence %.3f\n", confidence);
        cpxPrintToConsole(LOG_TO_CRTP, "Bounding Box: [%.3f, %.3f, %.3f, %.3f]\n",
                          bbox[0], bbox[1], bbox[2], bbox[3]);
    }
    else
    {
        cpxPrintToConsole(LOG_TO_CRTP, "No object detected. Confidence: %.3f\n", confidence);
    }

    // Prepare the image for Wi-Fi streaming
    if (wifiClientConnected == 1)
    {
        // Encode image using JPEG encoder
        pi_buffer_t buffer;
        pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, cameraBufferFull);
        pi_buffer_set_format(&buffer, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
        jpeg_encoder_process(&jpeg_encoder, &buffer, &jpeg_data, &jpegSize);

        // Send the image over Wi-Fi
        createImageHeaderPacket(&txp, headerSize + jpegSize + footerSize, JPEG_ENCODING);
        cpxSendPacketBlocking(&txp);

        // Send JPEG header
        memcpy(txp.data, header.data, headerSize);
        txp.dataLength = headerSize;
        cpxSendPacketBlocking(&txp);

        // Send JPEG data
        sendBufferViaCPX(&txp, (uint8_t *)jpeg_data.data, jpegSize);

        // Send JPEG footer
        memcpy(txp.data, footer.data, footerSize);
        txp.dataLength = footerSize;
        cpxSendPacketBlocking(&txp);
    }

    // Start capturing next image
    pi_camera_capture_async(&camera, cameraBufferFull, CAM_FULL_WIDTH * CAM_FULL_HEIGHT,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
}

static void capture_done_cb(void *arg)
{
    xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

static int open_camera(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;
    pi_himax_conf_init(&cam_conf);
    cam_conf.format = PI_CAMERA_QVGA; // 324x244
    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    // Rotate image if necessary
    uint8_t set_value = 3;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);

    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    return 0;
}

void hb_task(void *parameters)
{
    (void)parameters;

    // Initialize the LED pin
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);

    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    while (1)
    {
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
        vTaskDelay(xDelay);
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
        vTaskDelay(xDelay);
    }
}

void rx_task(void *parameters)
{
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
            break;
        }
    }
}

void camera_task(void *parameters)
{
    vTaskDelay(2000);

#ifdef SETUP_WIFI_AP
    setupWiFi();
#endif

    cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");

    // Initialize camera
    if (open_camera(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Camera opened\n");

    // Allocate buffers
    cameraBufferFull = (unsigned char *)pmsis_l2_malloc(CAM_FULL_WIDTH * CAM_FULL_HEIGHT);
    cameraBufferResized = (unsigned char *)pmsis_l2_malloc(CAM_WIDTH * CAM_HEIGHT);
    Output_1 = (signed short *)pmsis_l2_malloc(sizeof(signed short));
    Output_2 = (signed char *)pmsis_l2_malloc(4 * sizeof(signed char));
    jpeg_data.data = pmsis_l2_malloc(1024 * 15);
    header.data = pmsis_l2_malloc(1024);
    footer.data = pmsis_l2_malloc(10);

    // Check for allocation failures
    if (!cameraBufferFull || !cameraBufferResized || !Output_1 || !Output_2 ||
        !jpeg_data.data || !header.data || !footer.data)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Memory allocation failed\n");
        pmsis_exit(-1);
    }

    // Initialize neural network cluster
    pi_cluster_conf_init(&cluster_conf);
    pi_open_from_conf(&cluster_dev, &cluster_conf);
    pi_cluster_open(&cluster_dev);
    task = pmsis_l2_malloc(sizeof(struct pi_cluster_task));
    memset(task, 0, sizeof(struct pi_cluster_task));
    task->entry = &RunNetwork;
    task->stack_size = STACK_SIZE;
    task->slave_stack_size = SLAVE_STACK_SIZE;
    task->arg = NULL;

    // Construct the neural network
    int ret = __PREFIX(CNN_Construct)();
    if (ret)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to construct CNN\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Neural network constructed\n");

    // Initialize JPEG encoder
    struct jpeg_encoder_conf enc_conf;
    jpeg_encoder_conf_init(&enc_conf);
    enc_conf.width = CAM_FULL_WIDTH;
    enc_conf.height = CAM_FULL_HEIGHT;
    enc_conf.flags = 0;

    if (jpeg_encoder_open(&jpeg_encoder, &enc_conf))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to initialize JPEG encoder\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "JPEG encoder initialized\n");

    // Prepare JPEG header and footer
    jpeg_encoder_header(&jpeg_encoder, &header, &headerSize);
    jpeg_encoder_footer(&jpeg_encoder, &footer, &footerSize);

    // We're reusing the same packet, so initialize the route once
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);

    // Start first image capture
    pi_camera_capture_async(&camera, cameraBufferFull, CAM_FULL_WIDTH * CAM_FULL_HEIGHT,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

    while (1)
    {
        pi_yield();
    }

    // Destruct the neural network
    __PREFIX(CNN_Destruct)();
}

int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    // Initialize UART for console output
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

    cpxPrintToConsole(LOG_TO_CRTP, "-- Neural Network and Wi-Fi Streaming Example --\n");

    // Create event group
    evGroup = xEventGroupCreate();

    // Create tasks
    BaseType_t xTask;

    xTask = xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Heartbeat task did not start!\n");
        pmsis_exit(-1);
    }

    xTask = xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start!\n");
        pmsis_exit(-1);
    }

    xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "RX task did not start!\n");
        pmsis_exit(-1);
    }

    // Start the scheduler
    vTaskStartScheduler();

    return 0;
}
