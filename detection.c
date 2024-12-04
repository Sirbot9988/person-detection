/**
 * @file detection.c
 */

#include "detection.h"
#include "bsp/camera/himax.h"
#include "bsp/ai_deck.h"
#include "bsp/buffer.h"
#include "bsp/transport/nina_w10.h"
#include "detectionKernels.h"
#include "gaplib/ImgIO.h"
#include "pmsis.h"
#include "stdio.h"
#include "bsp/bsp.h"
#include "cpx.h"

#define CAM_FULL_WIDTH 324
#define CAM_FULL_HEIGHT 244

#define CAM_WIDTH 80
#define CAM_HEIGHT 48

#define CHANNELS 1
#define IO RGB888_IO
#define CAT_LEN sizeof(uint32_t)

#define __XSTR(__s) __STR(__s)
#define __STR(__s) #__s

static pi_task_t task1;
static pi_task_t task2;
static unsigned char *cameraBufferFull;
static unsigned char *cameraBufferResized;
static signed short *Output_1; // Confidence score
static signed char *Output_2;  // Bounding Box

static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;

AT_HYPERFLASH_FS_EXT_ADDR_TYPE __PREFIX(_L3_Flash) = 0;

#define IMG_ORIENTATION 0x0101
uint8_t set_value2 = 1;
uint8_t set_value3 = 16;
uint8_t set_value4 = 1;

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

static void cam_handler(void *arg)
{
    cpxPrintToConsole(LOG_TO_CRTP, "cam_handler called\n");

    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // call image resizer function
    cpxPrintToConsole(LOG_TO_CRTP, "Resizing image\n");
    resize_image(cameraBufferFull, cameraBufferResized, CAM_FULL_WIDTH, CAM_FULL_HEIGHT, CAM_WIDTH, CAM_HEIGHT);

    // inference
    cpxPrintToConsole(LOG_TO_CRTP, "Running neural network inference\n");
    int cluster_ret = pi_cluster_send_task_to_cl(&cluster_dev, task);
    if (cluster_ret)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to send task to cluster: %d\n", cluster_ret);
    }
    else
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Cluster task executed successfully\n");
    }

    // print raw outputs for debugging
    cpxPrintToConsole(LOG_TO_CRTP, "Raw Output_1 value: %d\n", Output_1[0]);
    cpxPrintToConsole(LOG_TO_CRTP, "Raw Output_2 values: %d %d %d %d\n",
                      Output_2[0], Output_2[1], Output_2[2], Output_2[3]);

    // Process neural network outputs
    cpxPrintToConsole(LOG_TO_CRTP, "Processing neural network outputs\n");
    float confidence = (float)Output_1[0] / 32768.0f;  // Map to [-1, 1]
    confidence = (confidence + 1.0f) / 2.0f;           // Map to [0, 1]
    cpxPrintToConsole(LOG_TO_CRTP, "Confidence: %.3f\n", confidence);
    float threshold = 0.5f;
    if (confidence >= threshold)
    {
        // if object detected has higher confidence than threshhold
        char bbStr[100] = "";
        for (int i = 0; i < 4; i++)
        {
            // Is output2 
            float bbox_value = (float)Output_2[i] / 128.0f; // Map to [-1, 1]
            bbox_value = (bbox_value + 1.0f) / 2.0f;        // Map to [0, 1]
            char bbtemp[50];
            sprintf(bbtemp, "%.3f ", bbox_value);
            strcat(bbStr, bbtemp);
        }
        cpxPrintToConsole(LOG_TO_CRTP, "Bounding Box: %s\n", bbStr);
    }
    else
    {
        cpxPrintToConsole(LOG_TO_CRTP, "No object detected.\n");
    }

    // Start capturing next image
    pi_camera_capture_async(&camera, cameraBufferFull, CAM_FULL_WIDTH * CAM_FULL_HEIGHT,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
}

static int open_camera(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;
    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QVGA; // Set to QVGA (324x244)
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
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
        return -1;
    }

    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);
    return 0;
}

// Functions and init for LED toggle
#define LED_PIN 2
static pi_device_t led_gpio_dev;
void hb_task(void *parameters)
{
    (void)parameters;
    char *taskname = pcTaskGetName(NULL);

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

int detection()
{
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQ_FC * 1000 * 1000);
    // pi_freq_set(PI_FREQ_DOMAIN_CL, FREQ_CL*1000*1000);
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    // debug uart connections
    struct pi_uart_conf uart_conf;
    struct pi_device device;
    pi_uart_conf_init(&uart_conf);
    uart_conf.baudrate_bps = 115200;

    // Start LED toggle
    BaseType_t xTask;
    xTask = xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    pi_open_from_conf(&device, &uart_conf);
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);

    cpxPrintToConsole(LOG_TO_CRTP, "*** Detection ***\n");

    cpxPrintToConsole(LOG_TO_CRTP, "Starting to open camera\n");

    if (open_camera(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return -1;
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Opened Camera\n");

    cameraBufferFull = (unsigned char *)pmsis_l2_malloc((CAM_FULL_WIDTH * CAM_FULL_HEIGHT) * sizeof(unsigned char));
    if (cameraBufferFull == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for full camera buffer\n");
        return 1;
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for full camera buffer\n");

    cameraBufferResized = (unsigned char *)pmsis_l2_malloc((CAM_WIDTH * CAM_HEIGHT) * sizeof(unsigned char));
    if (cameraBufferResized == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for resized camera buffer\n");
        return 1;
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for resized camera buffer\n");

    Output_1 = (signed short *)pmsis_l2_malloc(1 * sizeof(signed short));
    if (Output_1 == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for confidence score\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for output 1\n");

    Output_2 = (signed char *)pmsis_l2_malloc(4 * sizeof(signed char));
    if (Output_2 == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for bounding box\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for output 2\n");

    /* Configure CNN task */
    pi_cluster_conf_init(&cluster_conf);
    pi_open_from_conf(&cluster_dev, (void *)&cluster_conf);
    pi_cluster_open(&cluster_dev);
    task = pmsis_l2_malloc(sizeof(struct pi_cluster_task));
    if (!task)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for task\n");
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for task\n");

    memset(task, 0, sizeof(struct pi_cluster_task));
    task->entry = &RunNetwork;
    task->stack_size = STACK_SIZE;             // defined in makefile
    task->slave_stack_size = SLAVE_STACK_SIZE; // "
    task->arg = NULL;

    /* Construct CNN */
    int ret = __PREFIX(CNN_Construct)();
    if (ret)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to construct CNN with %d\n", ret);
        pmsis_exit(-5);
    }
    cpxPrintToConsole(LOG_TO_CRTP, "Constructed CNN\n");

    // Start first image capture
    pi_camera_capture_async(&camera, cameraBufferFull, CAM_FULL_WIDTH * CAM_FULL_HEIGHT,
                            pi_task_callback(&task1, cam_handler, NULL));
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

    while (1)
    {
        pi_yield();
    }

    /* Destruct CNN */
    __PREFIX(CNN_Destruct)();

    return 0;
}

int main(void)
{
    pi_bsp_init();
    return pmsis_kickoff((void *)detection);
}
