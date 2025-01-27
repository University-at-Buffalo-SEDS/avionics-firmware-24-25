#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

#if !defined(HAL_CAN_MODULE_ENABLED)
#define HAL_CAN_MODULE_ENABLED
#endif

#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

#include <stm32g4xx_hal_fdcan.h>

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void FDCAN_Config(void);
static void MX_FDCAN2_Init(void);

FDCAN_HandleTypeDef hfdcan2;

uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

void setup() {
    // Initialize serial communication
    #if defined(USBCON) && defined(USBD_USE_CDC)
        usb_serial.begin();
    #else
        Serial.begin(9600);
    #endif

    while (!Serial) { ; }

    Wire.begin();

    // Setup the device

    // Setup CAN
    MX_FDCAN2_Init();
    FDCAN_Config();

    // Create mutex for shared data

    vTaskStartScheduler();
}

void loop() {
    // Empty. Tasks are now scheduled by FreeRTOS.
}

//Generated by STM32CubeIDE
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  HAL_StatusTypeDef err = HAL_FDCAN_Init(&hfdcan2);
  if (err != HAL_OK)
  {
    Serial.print(F("FDCAN Init failed: "));
    Serial.print(err);
    Error_Handler();
  }
}

static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;
  HAL_StatusTypeDef err = HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
  if (err != HAL_OK)
  {
    Serial.print(F("FDCAN filter config failed: "));
    Serial.print(err);
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  err = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  if (err != HAL_OK)
  {
    Serial.print(F("FDCAN global filter config failed: "));
    Serial.print(err);
    Error_Handler();
  }

  /* Start the FDCAN module */
  err = HAL_FDCAN_Start(&hfdcan2);
  if (err != HAL_OK)
  {
    Serial.print(F("FDCAN global filter config failed: "));
    Serial.print(err);
    Error_Handler();
  }

  err = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  if (err != HAL_OK)
  {
    Serial.print(F("FDCAN global filter config failed: "));
    Serial.print(err);
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x321;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}
