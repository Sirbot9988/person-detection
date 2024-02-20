/**
 * @file detection.c
 */

#include "detection.h"
#include "bsp/camera/himax.h"
#include "bsp/transport/nina_w10.h"
#include "detectionKernels.h"
#include "gaplib/ImgIO.h"
#include "pmsis.h"
#include "stdio.h"
#include "bsp/bsp.h"
#include "cpx.h"

#define CAM_WIDTH 324
#define CAM_HEIGHT 244

#define CHANNELS 1
#define IO RGB888_IO
#define CAT_LEN sizeof(uint32_t)

#define __XSTR(__s) __STR(__s)
#define __STR(__s) #__s

static pi_task_t task1;
static pi_task_t task2;
static unsigned char *cameraBuffer;
static signed short *Output_1; // Class
static signed char *Output_2; // Bounding Box

const char *class_mapping[] = {"none", "ball", "cone"};

static struct pi_device camera;
static struct pi_device cluster_dev;
static struct pi_cluster_task *task;
static struct pi_cluster_conf cluster_conf;

AT_HYPERFLASH_FS_EXT_ADDR_TYPE __PREFIX(_L3_Flash) = 0;

#define IMG_ORIENTATION 0x0101


static void RunNetwork()
{
  __PREFIX(CNN)(cameraBuffer, Output_1, Output_2);
}

static void cam_handler(void *arg)
{

  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

  /* Run inference */
  pi_cluster_send_task_to_cl(&cluster_dev, task);

  short int max_value = Output_1[0];
  int max_index = 0;
  char classStr[100] = ""; 
  for (int i = 0; i < 3; i++) {
      if (Output_1[i] > max_value) {
          max_value = Output_1[i];
          max_index = i;
      }
      int Output_1_int = (int)Output_1[i];
      char classtemp[50]; 
      sprintf(classtemp, "%d ", Output_1_int); 
      strcat(classStr, classtemp); 
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Output1: %s\n", classStr);
  cpxPrintToConsole(LOG_TO_CRTP, "Class Detected: %s\n", class_mapping[max_index]);

  if (max_index != 0) { 
      char bbStr[100] = ""; 
      for (int i = 0; i < 4; i++) {
          float Output_2_int = (float)Output_2[i]*0.00592281;
          char bbtemp[50]; 
          sprintf(bbtemp, "%.3f ", Output_2_int); 
          strcat(bbStr, bbtemp); 
      }
      cpxPrintToConsole(LOG_TO_CRTP, "Bounding Box: %s\n", bbStr);
  }

  // himax_set_register(0x2100, 0x1); // AE_CTRL
  // himax_set_register(0x0205, 0x10); // ANALOG_GLOBAL_GAIN: 0x10 = 2x, 0x20 = 4x
  
  // // This is needed for the camera to actually update its registers.
  // himax_set_register(0x0104, 0x1);
  
  pi_camera_capture_async(&camera, cameraBuffer, CAM_HEIGHT * CAM_WIDTH, pi_task_callback(&task1, cam_handler, NULL));
  pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
}

static int open_camera(struct pi_device *device)
{

  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QVGA;

  pi_open_from_conf(device, &cam_conf);
  if (pi_camera_open(device))
    return -1;

  pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
  uint8_t set_value = 3;
  uint8_t reg_value;
  pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
  pi_time_wait_us(1000000);
  pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);

  if (set_value != reg_value)
  {
    cpxPrintToConsole(LOG_TO_CRTP,"Failed to rotate camera image\n");
    return -1;
  }
              
  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

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
  pi_freq_set(PI_FREQ_DOMAIN_FC, FREQ_FC*1000*1000);
  //pi_freq_set(PI_FREQ_DOMAIN_CL, FREQ_CL*1000*1000);
  pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

  // For debugging
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
  cpxPrintToConsole(LOG_TO_CRTP,"Opened Camera\n");

  cameraBuffer = (unsigned char *)pmsis_l2_malloc((CAM_HEIGHT * CAM_WIDTH) * sizeof(unsigned char));
  if (cameraBuffer == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed Allocated memory for camera buffer\n");
    return 1;
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for camera buffer\n");

  Output_1 = (signed short *)pmsis_l2_malloc(3 * sizeof(signed short));
  if (Output_1 == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for bounding box\n");
    pmsis_exit(-1);
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Allocated memory for output 1\n");

  Output_2 = (signed char *)pmsis_l2_malloc(4 * sizeof(signed char));
  if (Output_2 == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate memory for class\n");
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
    cpxPrintToConsole(LOG_TO_CRTP, "failed to allocate memory for task\n");
  }
  cpxPrintToConsole(LOG_TO_CRTP,"Allocated memory for task\n");

  memset(task, 0, sizeof(struct pi_cluster_task));
  task->entry = &RunNetwork;
  task->stack_size = STACK_SIZE;             // defined in makefile
  task->slave_stack_size = SLAVE_STACK_SIZE; // "
  task->arg = NULL;

  /* Construct CNN */
  int ret = __PREFIX(CNN_Construct)();
  if (ret)
  {
    cpxPrintToConsole(LOG_TO_CRTP,"Failed to construct CNN with %d\n", ret);
    pmsis_exit(-5);
  }
  cpxPrintToConsole(LOG_TO_CRTP,"Constructed CNN\n");

  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
  // himax_set_register(0x2100, 0x1); // AE_CTRL
  // himax_set_register(0x0205, 0x10); // ANALOG_GLOBAL_GAIN: 0x10 = 2x, 0x20 = 4x
  
  // // This is needed for the camera to actually update its registers.
  // himax_set_register(0x0104, 0x1);
  pi_camera_capture_async(&camera, cameraBuffer, CAM_HEIGHT * CAM_WIDTH , pi_task_callback(&task1, cam_handler, NULL));
  pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);

  while (1)
  {
    pi_yield();
  }

  /* Destruct CNN */
  __PREFIX(CNN_Destruct)
  ();

  return 0;
}

int main(void)
{
  pi_bsp_init();
  return pmsis_kickoff((void *)detection);
}
