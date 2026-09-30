#include <DJIMotorCtrlESP.hpp>
#include <HXC_TWAI.hpp>

#define TX_PIN D2
#define RX_PIN D3

#define SHINKU_PIN_1 D4 // 排気用
#define SHINKU_PIN_2 D5 // 吸気用
#define SHINKU_PIN_3 D6

#define RECEIVE_ID 0x310

uint16_t ID = 0;
int8_t data[8] = {0};

TaskHandle_t motor_control_handle = NULL;
twai_handle_t receive_handle = NULL;

HXC_TWAI CAN_BUS(D0, D1, CAN_RATE_1MBIT);

M3508_P19 MOTOR1(&CAN_BUS, 1);
M3508_P19 MOTOR2(&CAN_BUS, 2);
M3508_P19 MOTOR3(&CAN_BUS, 3);
M3508_P19 MOTOR4(&CAN_BUS, 4);

void receive(void *pvParameters)
{
  for (;;)
  {
    twai_message_t receiveframe;
    if (twai_receive_v2(receive_handle, &receiveframe, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      if (receiveframe.identifier == RECEIVE_ID)
      {

        ID = receiveframe.identifier;
        data[0] = receiveframe.data[0]; // 命令の識別子
        data[1] = receiveframe.data[1]; // 命令の数値(足回りの場合は倍率に使用)
        data[2] = receiveframe.data[2]; // vector1
        data[3] = receiveframe.data[3]; // vector2
        data[4] = receiveframe.data[4]; // vector3
        data[5] = receiveframe.data[5]; // vector4
        data[6] = receiveframe.data[6]; // ゴミ
        data[7] = receiveframe.data[7]; // ゴミ
        Serial.printf("ID = %x, data1 = %d, data2 = %d, data3 = %d, data4 = %d, data5 = %d, data6 = %d, data7 = %d, data8 = %d\n", ID, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        // xTaskNotifyGive(motor_control_handle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void motor_control(void *pvParameters)
{

  for (;;)
  {
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    MOTOR1.set_speed(map(data[2] * data[1] * -1, -100, 100, -200, 200));
    MOTOR2.set_speed(map(data[3] * data[1], -100, 100, -200, 200));
    MOTOR3.set_speed(map(data[4] * data[1] * -1, -100, 100, -200, 200));
    MOTOR4.set_speed(map(data[5] * data[1], -100, 100, -200, 200));

    switch (data[0])
    {

    case 2:

      digitalWrite(D4, HIGH);
      digitalWrite(D5, HIGH);
      digitalWrite(D6, HIGH);
      break;

    case 3:
      digitalWrite(D4, LOW);
      digitalWrite(D5, LOW);
      digitalWrite(D6, LOW);
      break;

    case 4:
      digitalWrite(D4, LOW);
      digitalWrite(D5, HIGH);
      break;

    case 5:
      digitalWrite(D4, HIGH);
      digitalWrite(D5, LOW);
      break;

    case 6:
      digitalWrite(D4, LOW);
      digitalWrite(D5, LOW);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{

  Serial.begin(115200);

  vTaskDelay(pdMS_TO_TICKS(200));

  if (CAN_BUS.setup() == ESP_OK)
  {
    Serial.println("CANセットアップ完了");
  }
  else
    Serial.println("失敗！！");

  MOTOR1.setup();
  MOTOR2.setup();
  MOTOR3.setup();
  MOTOR4.setup();

  pinMode(SHINKU_PIN_1, OUTPUT);
  pinMode(SHINKU_PIN_2, OUTPUT);
  // pinMode(SHINKU_PIN_3, OUTPUT);

  digitalWrite(SHINKU_PIN_1, LOW);
  digitalWrite(SHINKU_PIN_2, LOW);
  // digitalWrite(SHINKU_PIN_3, LOW);

  /***********************************CAN関連********************************************/
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(1, (gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t ret = twai_driver_install_v2(&g_config, &t_config, &f_config, &receive_handle);
  if (ret == ESP_OK)
    Serial.println("インストール完了");
  else
    Serial.println("インストール失敗");
  ret = twai_start_v2(receive_handle);
  if (ret == ESP_OK)
    Serial.println("CANスタート完了");
  else
    Serial.println("CANスタート失敗");
  /**************************************************************************************/

  xTaskCreateUniversal(
      receive,
      "receive",
      8192,
      NULL,
      2,
      NULL,
      PRO_CPU_NUM);

  xTaskCreateUniversal(
      motor_control,
      "motor_control",
      4096,
      NULL,
      1,
      &motor_control_handle,
      PRO_CPU_NUM);
}

void loop()
{

  twai_status_info_t status_info;

  // ステータス情報を取得
  esp_err_t ret = twai_get_status_info(&status_info);

  // state が TWAI_STATE_BUS_OFF かどうか判定
  if (status_info.state == TWAI_STATE_STOPPED || status_info.state == TWAI_STATE_BUS_OFF)
  {
    Serial.println("バスがオフです。リカバリを開始します");

    ret = twai_initiate_recovery();
    if (ret != ESP_OK)
    {
      Serial.printf("リカバリ開始失敗: %s\n", esp_err_to_name(ret));
    }
    Serial.printf("リカバリの開始に成功しました。\n");

    ret = twai_start();
    if (ret == ESP_OK)
    {
      Serial.println("CANドライバの再起動に成功しました");
    }
    else
    {
      Serial.printf("再スタート失敗: %s\n", esp_err_to_name(ret));
    }
  } // Serial.println(status_info.state);
  vTaskDelay(pdMS_TO_TICKS(1));
}