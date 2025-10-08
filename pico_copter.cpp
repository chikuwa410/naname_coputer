#include "pico_copter.hpp"

//グローバル変数
uint8_t Arm_flag=0;
uint8_t Red_flag = 0;
semaphore_t sem;


int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  printf("#start setup\r\n"); 

  //Initialize stdio for Pico
  stdio_init_all();
  printf("#init pico\r\n");

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
  // Kalman_init();
  // initialize_Altitude();
  // printf("#init control\r\n");

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

  Arm_flag=1;
  
  while(1)
  {
    // printf("Arm_flag:%d LockMode:%d\n",Arm_flag, LockMode);
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
