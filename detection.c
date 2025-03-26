/* 
CURRENT ISSUES: 
The bounding box is not being drawn on the image. The values of Output_1 and Output_2 are not changing.
Connection to radio is not great. Possibly because the large amount of debug outputs and its frequency being sent may be limiting bandwidth. 
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

/* incl if code does not need L3 Flash . Otherwise detectionKernel will call L3 Flash leading to unknown refernece error. */
AT_HYPERFLASH_FS_EXT_ADDR_TYPE detection_L3_Flash = 0;

#define CAM_FULL_WIDTH   324
#define CAM_FULL_HEIGHT  244
#define MODEL_WIDTH        81
#define MODEL_HEIGHT       61
#define IMG_ORIENTATION  0x0101

#ifndef STACK_SIZE
#define STACK_SIZE       (1024 * 2)
#endif

#ifndef SLAVE_STACK_SIZE
#define SLAVE_STACK_SIZE (1024)
#endif

#define LED_PIN          2
#define THRESHOLD        0.5f
#define JPEG_BUFFER_SIZE (50 * 1024) // Adjust if needed

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

static int wifiConnected = 0;
static int wifiClientConnected = 0;

static pi_task_t task1;
static CPXPacket_t rxp;
static CPXPacket_t txp;

static unsigned char *cameraBufferFull;
static unsigned char *cameraBufferResized; 
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
static void DrawRectangleNormalized(unsigned char *img, int img_w, int img_h,
    float x_center, float y_center, float box_width, float box_height)
{
    // Convert normalized coordinates to pixel coordinates.
    int x_min = (int)((x_center - box_width / 2.0f) * img_w);
    int y_min = (int)((y_center - box_height / 2.0f) * img_h);
    int x_max = (int)((x_center + box_width / 2.0f) * img_w);
    int y_max = (int)((y_center + box_height / 2.0f) * img_h);

    // Clamp coordinates to image boundaries.
    if (x_min < 0) x_min = 0;
    if (y_min < 0) y_min = 0;
    if (x_max >= img_w) x_max = img_w - 1;
    if (y_max >= img_h) y_max = img_h - 1;

    // Draw top and bottom edges.
    for (int x = x_min; x <= x_max; x++)
    {
    if (y_min >= 0 && y_min < img_h)
    img[y_min * img_w + x] = 255;  // Top edge
    if (y_max >= 0 && y_max < img_h)
    img[y_max * img_w + x] = 255;  // Bottom edge
    }

    // Draw left and right edges.
    for (int y = y_min; y <= y_max; y++)
    {
    if (x_min >= 0 && x_min < img_w)
    img[y * img_w + x_min] = 255;  // Left edge
    if (x_max >= 0 && x_max < img_w)
    img[y * img_w + x_max] = 255;  // Right edge
    }
}

static void capture_done_cb(void *arg)
{
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
            printf( "Wi-Fi connected (%u.%u.%u.%u)\n",
                              wifiCtrl->data[0], wifiCtrl->data[1],
                              wifiCtrl->data[2], wifiCtrl->data[3]);
            wifiConnected = 1;
            break;
        case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
            printf( "Wi-Fi client connection status: %u\n", wifiCtrl->data[0]);
            wifiClientConnected = wifiCtrl->data[0];
            break;
        default:
            printf( "Unknown Wi-Fi CTRL command: %u\n", wifiCtrl->cmd);
            break;
        }
    }
    return;
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
    printf( "Setting up Wi-Fi AP\n");

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
    pi_himax_conf_init(&cam_conf);
    cam_conf.format = PI_CAMERA_QVGA;

    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);
    pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        printf( "Failed to rotate camera image\n");
        return -1;
    }

    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    return 0;
}

static void RunNetwork()
{
    __PREFIX(CNN)((signed char *)cameraBufferResized, Output_1);
}

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

static void cam_handler(void *arg)
{
    (void)arg;
    printf( "cam_handler called\n");
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // Resize for inference
    printf( "Resizing image\n");
    resize_image(cameraBufferFull, cameraBufferResized, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, MODEL_WIDTH, MODEL_HEIGHT);

    // Run inference
    printf( "Running neural network inference\n");
    pi_cluster_send_task_to_cl(&cluster_dev, task);

    // Process outputs (Q7)
    printf( "Processing neural network outputs\n");
    float x_center = ((float)Output_1[0] / 128.0f + 1.0f) / 2.0f;
    float y_center = ((float)Output_1[1] / 128.0f + 1.0f) / 2.0f;
    float box_width = ((float)Output_1[2] / 128.0f + 1.0f) / 2.0f;
    float box_height = ((float)Output_1[3] / 128.0f + 1.0f) / 2.0f;

    printf( "Detected Object: x_center=%.3f, y_center=%.3f, box_width=%.3f, box_height=%.3f\n",
                      x_center, y_center, box_width, box_height);

    // bbox is written on cameraBufferFull for drawing onto the image
    DrawRectangleNormalized(cameraBufferFull, CAM_FULL_WIDTH, CAM_FULL_HEIGHT,
        x_center, y_center, box_width, box_height);
    if (wifiClientConnected == 1)
    {
        printf( "Encoding image as JPEG\n");

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
        printf( "encoding=%d ms (%d bytes), transfer=%d ms\n",
                          encodingTime, imgSize, transferTime);
    }
    else
    {
        printf( "No Wi-Fi client connected, skipping image send\n");
    }
    vTaskDelay(1000); //add delay to let other tasks catch up
    // Start next capture
    printf( "Starting next image capture\n");
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
    vTaskDelay(2000); 

    setupWiFi();

    printf( "Starting camera task...\n");

    uint32_t resolution = CAM_FULL_WIDTH * CAM_FULL_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);

    cameraBufferFull = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (!cameraBufferFull)
    {
        printf( "Failed to allocate cameraBufferFull\n");
        return;
    }

    cameraBufferResized = (unsigned char *)pmsis_l2_malloc(MODEL_WIDTH * MODEL_HEIGHT * sizeof(unsigned char));
    if (!cameraBufferResized)
    {
        printf( "Failed to allocate cameraBufferResized\n");
        return;
    }

    Output_1 = (signed char *)pmsis_l2_malloc(4*sizeof(signed char)); //size to number of BB coordinates
    if (!Output_1)
    {
        printf( "Failed to allocate Output_1\n");
        return;
    }


    // Initialize cluster
    pi_cluster_conf_init(&cluster_conf);
    pi_open_from_conf(&cluster_dev, &cluster_conf);
    pi_cluster_open(&cluster_dev);

    task = (struct pi_cluster_task *)pmsis_l2_malloc(sizeof(struct pi_cluster_task));
    if (!task)
    {
        printf( "Failed to allocate cluster task\n");
        return;
    }
    memset(task, 0, sizeof(struct pi_cluster_task));
    task->entry = &RunNetwork;
    task->stack_size = STACK_SIZE;
    task->slave_stack_size = SLAVE_STACK_SIZE;

    int ret = __PREFIX(CNN_Construct)();
    if (ret)
    {
        printf( "Failed to construct CNN with %d\n", ret);
        pmsis_exit(-5);
    }
    printf( "Constructed CNN\n");

    // Initialize JPEG encoder
    struct jpeg_encoder_conf enc_conf;
    jpeg_encoder_conf_init(&enc_conf);
    enc_conf.width = CAM_FULL_WIDTH;
    enc_conf.height = CAM_FULL_HEIGHT;
    enc_conf.flags = 0;

    if (jpeg_encoder_open(&encoder_struct, &enc_conf))
    {
        printf( "Failed to initialize JPEG encoder\n");
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
        printf( "Failed to allocate memory for JPEG structures\n");
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

void start_example(void)
{
    struct pi_uart_conf conf;
    struct pi_device device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200;
    if (open_camera(&camera))
    {
        printf( "Failed to open camera\n");
        return;
    }
    printf( "Opened Camera\n");
    pi_open_from_conf(&device, &conf);
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed!\n");
        pmsis_exit(-1);
    }

    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);
    printf( "-- Detection JPEG example with Bounding Box --\n");
  
    

    evGroup = xEventGroupCreate();

    // Heartbeat task
    xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    // RX task
    xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Camera task
    if (xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 16,
                    NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        printf( "camera_task did not start!\n");
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