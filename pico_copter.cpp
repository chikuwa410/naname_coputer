#include "pico_copter.hpp"
extern float Phi, Theta; // ToFの値を角度補正するために姿勢データを使う

//グローバル変数
uint8_t Arm_flag=0;
uint8_t Red_flag = 0;
semaphore_t sem;

// Optical Flow ポインタ
PMW3901* flow = nullptr;


int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  printf("#start setup\r\n"); 

  //Initialize stdio for Pico
  stdio_init_all();
  printf("#init pico\r\n");

  //adc initial
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);

  //Initialize LSM9DS1
  imu_mag_init();
  printf("#init gyro\r\n");

  //Initialize Radio
  radio_init();
  printf("#init radio\r\n");

  //Initialize Variavle
  variable_init();
  printf("#init cariavle\r\n");

  //Initilize Control
  control_init();
  Kalman_init();
  // initialize_Altitude();
  printf("#init control\r\n");

  // //RGB LED off
  rgbled_off();
  
  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib= 0;
  pwm_init();
  printf("#init pwm\r\n");

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  //ToFセンサの初期化
  initialize_Altitude();
  Arm_flag=1;

  flow = new PMW3901(spi0, 16, 19, 18, 17);
  flow->pmw_init();

  if (flow != nullptr) { // nullptr チェックのみ
      printf("# optical flow ready\r\n");
  } else {
      printf("Error: Optical flow sensor init failed!\n");
  }

  while(1) 
  {

    tight_loop_contents(); 
    while (Logoutputflag==1){ 
      log_output(); 
    } 
  }  

      // ===== 終了処理 =====
    if (flow) {
        delete flow;
        flow = nullptr;
    }
  return 0;
}
