
#include "control.hpp"

bool first_run = true;
// alt control
// Kalman filter and alt sensor
float i2c_connect = 1;
float stick = 0;
float start_time = time_us_64();
uint64_t count_up = 0;
float Kalman_alt = 0;
float last_Kalman_alt = 0;
float auto_mode_count = 0;
float current_time;
float func_time;
float z_acc;
float base_dis_count = 0;
uint16_t Range;
int8_t status;
uint8_t Temp2;
uint8_t Temp;
uint8_t IntPol;
uint8_t val = 0;
uint8_t range_gbuf[16];
float range_flag = 0;
uint16_t altitude_count = 0;
// float stick;
float auto_mode = 0;
float ideal;
float hove_time = 0.0;
float flying_mode = 0;
float input = 0;
Matrix<float, 3, 3> lotate_mat = MatrixXf::Zero(3, 3);
float f_distance = 0;
float f_distance2 = 0;
float f_distance3 = 0;
float lotated_distance = 0;
Matrix<float, 1, 3> distance_mat = MatrixXf::Zero(1, 3);
Matrix<float, 1, 3> f_distance_mat = MatrixXf::Zero(1, 3);
// OpenMV通信用
char buffer[BUFFER_SIZE];
int buffer_index = 0;
uint8_t release_flag = 0;
uint8_t red_count = 0;
uint16_t loop_count = 0;
float x_1_dash;
float x_2_dash;
float y_1_dash;
float y_2_dash;
float x_diff = 0;
float angle_diff = 0;
int previous_gap_number = 0;
float line_number = 0;
float length = 0;
float x_diff_dash = 0;
float TOL_x_diff_dash = 0;
float TOL_y_diff_dash = 0;
float red_circle = 0;
float TOL_x_diff = 0;
float TOL_y_diff = 0;
float TOL_flag = 0;
float TOL_x_ref = 80;
float TOL_y_ref = 60;
float TOL_x_err = 0;
float TOL_y_err = 0;
float takeoff_counter = 0;
float landing_counter = 0;
float line_trace_flag = 0;
float gap_number = 0;
float T_merker_flag = 0;
float L_merker_flag = 0;
float TOL_x_alpha = 0;
float TOL_y_alpha = 0;
float x_alpha = 0;
float altitude = 0;
int length_count = 0;

//flip
int ahrs_flag=0;
int flip_flag1;
int flip_flag2;
float flip_p_ref=0 ,flip_q_ref=0, flip_r_ref=0;
float v_left=0, v_right=0;

//flow data
float vx, vy;
const float dt = 0.01;

// Sensor data
float Ax, Ay, Az, Wp, Wq, Wr, Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
float Acc_norm = 0.0;
float Line_range = 0.0;
float Line_velocity = 0.0;

// Initial data
float rate_limit = 180.0;

// // Rocking wings
float Rocking_timer = 0.0;
float rocking_wings(float stick);
uint8_t Flight_mode = 0;

// Times
float Elapsed_time = 0.0;
uint32_t S_time = 0, E_time = 0, D_time = 0, S_time2 = 0, E_time2 = 0, D_time2 = 0;
uint8_t red_state = 0;               // 状態保持
uint8_t red_count2=0;
int red_flag2=0;

// Counter
uint8_t AngleControlCounter = 0;
uint16_t RateControlCounter = 0;
uint16_t BiasCounter = 0;
uint16_t LedBlinkCounter = 0;
uint16_t LineTraceCounter = 0;
uint16_t Linetrace_counter_for_control = 0;
uint16_t plotnum = 0;
// Control
float FR_duty, FL_duty,MR_duty, ML_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float T_stick;
float T_ref_coe = 0.6;//0.7
float Pbias = 0.0, Qbias = 0.0, Rbias = 0.0;
float Phi_bias = 0.0, Theta_bias = 0.0, Psi_bias = 0.0;
float Phi, Theta, Psi = 0.0;
float Phi_ref = 0.0, Theta_ref = 0.0, Psi_ref = 0.0;
float Elevator_center = 0.0, Aileron_center = 0.0, Rudder_center = 0.0;
float Pref = 0.0, Qref = 0.0, Rref = 0.0;
float Phi_trim = 0;
float Theta_trim = 0;
float Psi_trim = 0.0;
const double pi = 3.14159;
float Line_trace_flag = 0;

//unique_mission(flipp)
uint16_t flip_count=0;
int red_flag1=0;

// Extended Kalman filter
Matrix<float, 7, 1> Xp = MatrixXf::Zero(7, 1);
Matrix<float, 7, 1> Xe = MatrixXf::Zero(7, 1);
Matrix<float, 6, 1> Z = MatrixXf::Zero(6, 1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q; // = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R; // = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7, 6> G;
Matrix<float, 3, 1> Beta;

// Log
uint16_t LogdataCounter = 0;
uint8_t Logflag = 0;
volatile uint8_t Logoutputflag = 0;
float Log_time = 0.0;
const uint8_t DATANUM = 33; // Log Data Number 33
const uint32_t LOGDATANUM = 47890;
float Logdata[LOGDATANUM] = {0.0};

// State Machine
uint8_t LockMode = 0;
float Disable_duty = 0.1;
float Flight_duty = 0.05; // 0.2///////////////// 0.18
uint8_t OverG_flag = 0;
unsigned short Linetrace_counter = 0;

// PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
PID u_pid;
PID v_pid;
PID y_pid;

Filter Range_filter;
Filter Angle_filter;
Filter Velocity_filter;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);
// 追加
void servo_control(void);
void led_control(void);
void linetrace(void);
void unique_mission(void);
void high_fall(void);


// alt control
void Merker_Hovering(void);
void Auto_fly(void);
void Auto_takeoff(void);
void Auto_landing(void);
void Hovering(void);
float lotate_altitude(float l_distance);
void lotate_altitude_init(float Theta, float Psi, float Phi);
void processReceiveData();
void uart_read(uint8_t *buffer, size_t len);
void receiveData(char c);
void takeoff_merker(void);
void landing_merker(void);
void send_data_via_uart(void);
void receiveData(char c);
void flow_sensor(void);


#define AVERAGE 2000
#define KALMANWAIT 6000

// // LED Control
void led_control(void)
{
  static uint16_t cnt = 0;

  if (Arm_flag == 0 || Arm_flag == 1) {
    rgbled_wait();
  }
  else if (Arm_flag == 2 && Flight_mode == NORMAL) {

    // 0→1 に変わった瞬間だけ赤LED
  if ((v_left > 1.5 && v_right > 1.5) || (Chdata[FLIP] > 200)) {
      rgbled_red();
    }
    else {
      rgbled_normal();
    }
  }

  else if (Arm_flag == 2 && Flight_mode == REDCIRCLE && (int)(red_circle == 0)) {
    rgbled_redcircle();
  }
  else if (Arm_flag == 2 && Flight_mode == REDCIRCLE && (int)(red_circle == 1)) {
    rgbled_red();
  }

  else if (Arm_flag == 3) {
    if (cnt == 0) rgbled_green();
    if (cnt == 50) rgbled_off();
    cnt++;
    if (cnt == 100) cnt = 0;
  }

  // 次回の判定用に状態を保存
}

// Main loop
// This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  


  static uint8_t led = 1;
  S_time = time_us_32();

  // 割り込みフラグリセット
  pwm_clear_irq(7);

  // Servo Control
  // servo_control();

  // LED Control
  led_control();

  //high fall
//  high_fall();


  if (Arm_flag == 0)
  {
    // motor_stop();
    Elevator_center = 0.0;
    Aileron_center = 0.0;
    Rudder_center = 0.0;
    Pbias = 0.0;
    Qbias = 0.0;
    Rbias = 0.0;
    Phi_bias = 0.0;
    Theta_bias = 0.0;
    Psi_bias = 0.0;
    return;
  }
  else if (Arm_flag == 1)
  {
    motor_stop();
    // Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      // Sensor Read
      sensor_read();
      // Aileron_center  += Chdata[3];
      // Elevator_center += Chdata[1];
      // Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if (BiasCounter < KALMANWAIT)
    {
      // Sensor Read
      sensor_read();
      if (BiasCounter == AVERAGE)
      {
        // Elevator_center = Elevator_center/AVERAGE;
        // Aileron_center  = Aileron_center/AVERAGE;
        // Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias / AVERAGE;
        Qbias = Qbias / AVERAGE;
        Rbias = Rbias / AVERAGE;
        Mx_ave = Mx_ave / AVERAGE;
        My_ave = My_ave / AVERAGE;
        Mz_ave = Mz_ave / AVERAGE;

        Xe(4, 0) = Pbias;
        Xe(5, 0) = Qbias;
        Xe(6, 0) = Rbias;
        Xp(4, 0) = Pbias;

        Xp(5, 0) = Qbias;
        Xp(6, 0) = Rbias;
        MN = Mx_ave;
        ME = My_ave;
        MD = Mz_ave;
      }

      AngleControlCounter++;
      if (AngleControlCounter == 4)
      {
        AngleControlCounter = 0;
        sem_release(&sem);
      }
      Phi_bias += Phi;
      Theta_bias += Theta;
      Psi_bias += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias = Phi_bias / KALMANWAIT;
      Theta_bias = Theta_bias / KALMANWAIT;
      Psi_bias = Psi_bias / KALMANWAIT;
      return;
    }
  }
  else if (Arm_flag == 2)
  {
    if (LockMode == 2)
    {
      if (lock_com() == 1)
      {
        LockMode = 3; // Disenable Flight
        led = 0;
        gpio_put(LED_PIN, led);
        return;
      }
      // Goto Flight
    }
    else if (LockMode == 3)
    {
      if (lock_com() == 0)
      {
        LockMode = 0;
        Arm_flag = 3;
      }
      return;
    }
    // LED Blink
    gpio_put(LED_PIN, led);
    if (Logflag == 1 && LedBlinkCounter < 100)
    {
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter = 0;
      led = !led;
    }

    // Rate Control (400Hz)
    rate_control();
    if (AngleControlCounter == 4)
    {
      AngleControlCounter = 0;
      // Angle Control (100Hz)
      sem_release(&sem);
    }
    // if(LineTraceCounter == 10)
    //{
    //   LineTraceCounter = 0;
    //   //linetrace (40Hz)
    //   if (Line_trace_flag == 1){
    //     linetrace();
    //   }
    // }
    AngleControlCounter++;
    LineTraceCounter++;
  }
  else if (Arm_flag == 3)
  {
    motor_stop();
    OverG_flag = 0;
    if (LedBlinkCounter < 10)
    {
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if (LedBlinkCounter < 100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else
      LedBlinkCounter = 0;

    // Get Stick Center
    Aileron_center = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center = Chdata[0];

    if (LockMode == 0)
    {
      if (lock_com() == 1)
      {
        LockMode = 1;
        return;
      }
      // Wait  output log
    }
    else if (LockMode == 1)
    {
      if (lock_com() == 0)
      {
        LockMode = 2; // Enable Flight
        Arm_flag = 2;
      }
      return;
    }

    if (logdata_out_com() == 1)
    {
      Arm_flag = 4;
      return;
    }
  }
  else if (Arm_flag == 4)
  {
    motor_stop();
    Logoutputflag = 1;
    // LED Blink
    rgbled_switch(led);
    gpio_put(LED_PIN, led);
    if (LedBlinkCounter < 400)
    {
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter = 0;
      led = !led;
    }
  }
  E_time = time_us_32();
  D_time = E_time - S_time;

      // ToFセンサから値を取得 
    // tof_poll(); 
    // uint16_t z_mm = 0; 
    // bool z_ok = tof_read_valid(&z_mm); 
    
    // // 1秒に1回だけシリアルモニタに出力する 
    // static uint32_t last_print_us = 0; 
    // uint32_t now = time_us_32(); 
    // if ((now - last_print_us) > 1000000) { 
    //   last_print_us = now; 
    //   if (z_ok) { 
    //     float corrected_z = (float)z_mm * cosf(Phi) * cosf(Theta); 
    //     printf("TOF Raw: %4u mm | Corrected: %4.1f mm\r\n", z_mm, corrected_z); 
    //   } else { 
    //     printf("TOF Raw: NA\r\n"); 
    //   } 
    // }
    

}


void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);
  Range_filter.set_parameter(0.08, 0.025);
  Angle_filter.set_parameter(0.08, 0.025);
  Velocity_filter.set_parameter(0.08, 0.025);


  // Rate control
  p_pid.set_parameter(0.032, 1000.0, 0.0125, 0.125, 0.0025); //ikaring(2.2, 5, 0.01) itocopter(2.5, 100, 0.009)
  q_pid.set_parameter(0.032, 1000.0, 0.0125, 0.125, 0.0025); //ikaring(1.5, 1, 0.01) itocopter(2.5, 100, 0.009)
  r_pid.set_parameter(0.21, 10000.0, 0.01, 0.125, 0.0025);  //ikaring(3.1, 1, 0.01) itocopter(3.5, 10, 0.009)
  // Angle control
  phi_pid.set_parameter(2.1, 1000.0, 0.013, 0.125, 0.01);   // 6.0  8.0,20,0.007      //1
  theta_pid.set_parameter(2.1, 1000.0, 0.013, 0.125, 0.01); // 6.0  8.0,20,0.007 // 1.6      
  psi_pid.set_parameter(0, 1000, 0, 0, 0.01);     //0 1000 0.01
  
  // Linetrace
  // velocity control
  v_pid.set_parameter(0.00025, 100000, 0.01, 0.125, 0.03); // 0.0002,100000,0.01
}

uint8_t lock_com(void)
{
  static uint8_t chatta = 0, state = 0;
  if (Chdata[2] < CH3MIN + 400 && Chdata[0] > CH1MAX - 400 && Chdata[3] < CH4MIN + 400 && Chdata[1] > CH2MAX - 400) //80
  {
    chatta++;
    if (chatta > 50)
    {
      chatta = 50;
      state = 1;
    }
  }
  else
  {
    chatta = 0;
    state = 0;
  }

  return state;
}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta = 0, state = 0;
  if (Chdata[4] < (CH5MAX + CH5MIN) * 0.5 && Chdata[2] < CH3MIN + 80 && Chdata[0] < CH1MIN + 80 && Chdata[3] > CH4MAX - 80 && Chdata[1] > CH2MAX - 80)
  {
    chatta++;
    if (chatta > 50)
    {
      chatta = 50;
      state = 1;
    }
  }
  else
  {
    chatta = 0;
    state = 0;
  }

  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_mr(0.0);
  set_duty_ml(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

// 高度制御の関数宣言
void lotate_altitude_init(float Theta, float Psi, float Phi)
{
  lotate_mat(0, 0) = cos(Theta) * cos(Psi);
  lotate_mat(0, 1) = cos(Theta) * sin(Psi);
  lotate_mat(0, 2) = -sin(Theta);
  lotate_mat(1, 0) = (sin(Phi) * sin(Theta) * cos(Psi)) - (cos(Phi) * sin(Psi));
  lotate_mat(1, 1) = (sin(Phi) * sin(Theta) * sin(Psi)) + (cos(Phi) * cos(Psi));
  lotate_mat(1, 2) = sin(Phi) * cos(Theta);
  lotate_mat(2, 0) = (cos(Phi) * sin(Theta) * cos(Psi)) + (sin(Phi) * sin(Psi));
  lotate_mat(2, 1) = (cos(Phi) * sin(Theta) * sin(Psi)) - (sin(Phi) * cos(Psi));
  lotate_mat(2, 2) = cos(Phi) * cos(Theta);
}

float lotate_altitude(float l_distance)
{
  // distance_mat(0,0) = 0;
  // distance_mat(0,1) = 0;
  distance_mat(0, 2) = l_distance;
  f_distance_mat = distance_mat * lotate_mat;
  f_distance = f_distance_mat(0, 0);
  f_distance2 = f_distance_mat(0, 1);
  f_distance3 = f_distance_mat(0, 2);

  return f_distance3;
}

// ホバリング
// void Hovering(void)
// {
  // 実験なので4秒
  // 本番はゴールを見つけたら着陸モード
  //  if (hove_time < 10)
  //  {
  //    input = alt_PID(ideal);
  //    T_ref = T_stick + (input);
  //    hove_time = hove_time + 0.01;
  //  }
  //  else{
  //    flying_mode = 3;
  //    //Auto_landing();
  //  }
  //  if (gap_number >=50){
  //    landing_counter = 1;
  //  }
//   input = alt_PID(ideal);
//   T_ref = T_stick + (input);
// }


// void Merker_Hovering(void){
//   //高度制御テスト用のコード
//   if(Flight_mode == HOVERING){
//     // auto_mode =1;
//     if(first_run){
//       // ideal = mu_Yn_est(1,0);
//       ideal = Kalman_alt;
//       T_stick = 0.7 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
//       // auto_mode_count = 1;
//       first_run = false;
//     }
//     if (count_up == 10) {
//       input = alt_PID(ideal);
//       T_ref = T_stick + (input);
//       count_up = 0;
//     }
//     count_up = count_up + 1;
//     printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
//   }
//   else{
//     first_run = true;
//     // auto_mode =0;
//     // auto_mode_count = 0;
//     T_ref = 0.7 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
//     // printf("T_ref:%9.6f ac:%9.6f\n",T_ref,auto_mode_count);
//     integral = 0;
//     integral_v = 0;
//     differential = 0;
//     differential_v = 0;
//   }

  // plotnum = plotnum+1;
  // if (plotnum==65000){
  //   plotnum = 0;
  // }

  // if (auto_mode ==1){
  //   // // 目標値との誤差
  //   // TOL_x_err = 0.008086 * (TOL_x_ref - TOL_x_diff);
  //   // TOL_y_err = 0.01 * (TOL_y_ref - TOL_y_diff);

  //   // Pref = phi_pid.update(TOL_x_err);
  //   // Qref = theta_pid.update(TOL_y_err);
  //   if(auto_mode_count == 0){
  //     auto_mode_count = 1;
  //     ideal = mu_Yn_est(1,0);
  //     T_stick = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
  //   }
  //   if (count_up == 10){
  //     input = alt_PID(ideal);
  //     T_ref = T_stick + (input);
  //     printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
  //     // printf("%9.6f %9.6f %9.6f %9.6f %9.6f f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input);
  //     // Hovering();
  //     count_up = 0;
  //   }
  //   count_up = count_up + 1;
  // }
  // printf("%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %d %9.6f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input,flying_mode,count_up,auto_mode_count);
// }

void servo_control(void)
{ 
  // if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //   payload_relese();
  // if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //   payload_hook();
  // if (Flight_mode == NORMAL){
  //   if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //     payload_relese();
  //   if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //     payload_hook();
  // }
  // else if(Flight_mode == REDCIRCLE){
  //   if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2)
  //     payload_relese();
  //   // if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2 && )
  //   //   payload_hook();
  // }
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_err, q_err, r_err;
  float p_ref, q_ref, r_ref;

  // Read Sensor Value
  sensor_read();

  // Mode SW
  // Chdata[MODE_SW]=1000;//本番はコメントにする
  // if (Chdata[MODE_SW]>1241)

  // if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  if ((Chdata[LOG] < 200) && (Chdata[HOVERING] < 200)&& i2c_connect == 1)
  {
    Flight_mode = NORMAL;
    Red_flag = 0;
    Rocking_timer = 0.0;
  }

  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] > 500) && (Chdata[ROCKING] < 200) && i2c_connect == 0)
  // {
  //   Flight_mode = NORMAL;
  //   Red_flag = 0;
  //   Rocking_timer = 0.0;
  // }

  else if ( (Chdata[LOG] < 200) && (Chdata[HOVERING] > 500)&& i2c_connect == 1)
  {
    Flight_mode = HOVERING;
    Red_flag = 0;
  }

  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LINETRACE] > 500) && (Chdata[ROCKING] < 200) && i2c_connect == 1)
  // {
    // Flight_mode = LINETRACE;
    // Red_flag = 0;
    // Rocking_timer = 0.0;
  // }
  // 赤丸検知モード
  // else if ((Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[HOVERING] < 200) && i2c_connect == 1)
  // {
  //   Flight_mode = REDCIRCLE;
  //   release_flag = 0;
  //   Rocking_timer = 0.0;
  // }


  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  // {
  //   Flight_mode = REDCIRCLE;
  //   release_flag = 0;
  //   Rocking_timer = 0.0;
  // }

  // else if ((Chdata[SERVO] > 500) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  // {
  //   Flight_mode = SERVO;
  //   Rocking_timer = 0.0;
  // }

  // Get Bias
  // Pbias = Xe(4, 0);
  // Qbias = Xe(5, 0);
  // Rbias = Xe(6, 0);
  
  // Control angle velocity
  
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;
  
  // Merker_Hovering();
  // Get reference
  //unique mission change
  if((Chdata[FLIP])>200){
    //unique_mission();
    p_ref =flip_p_ref;
    q_ref =flip_q_ref;
    r_ref =flip_r_ref;

  }
  else{

    if (Flight_mode == HOVERING) { 
        
      if (auto_mode_count == 0) {
        auto_mode_count = 1;
        ideal = Kalman_alt; // 目標高度ロック
        T_stick = (float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
      }

      // 40Hz（10回に1回）でカスケードPID実行
      if (count_up >= 10) {
        input = alt_PID(ideal); 
        count_up = 0;
      }
      count_up++;

      T_ref = T_stick + input; // 基準推力 ＋ PID補正値

    } else {
    // NORMALモード（手動操縦）
    auto_mode_count = 0;
    count_up = 0;
    input = 0;

    // 手動に戻る際、PIDの積分ゴミをリセット（ekf.hppのグローバル変数）
    integral = 0;
    integral_v = 0;
    differential = 0;
    differential_v = 0;

    T_ref = (float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

    // T_ref =(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

    // flip_count=0;
    
    // p_ref = Pref;
    // q_ref = Qref;
    // r_ref = Rref;
    }

  flip_count=0;
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  }

  // if (Flight_mode != LINETRACE)
  // if (Flight_mode != HOVERING)

  //Merker_Hovering();

  // //高度制御テスト用のコード
  // if(Flight_mode == HOVERING){
  //   auto_mode =1;
  // }
  // else{
  //   auto_mode =0;
  //   auto_mode_count = 0;
  //   T_ref = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  //   // printf("T_ref:%9.6f ac:%9.6f\n",T_ref,auto_mode_count);
  // }

  // plotnum = plotnum+1;
  // if (plotnum==65000){
  //   plotnum = 0;
  // }
  // if (auto_mode ==1){
  //   // // 目標値との誤差
  //   // TOL_x_err = 0.008086 * (TOL_x_ref - TOL_x_diff);
  //   // TOL_y_err = 0.01 * (TOL_y_ref - TOL_y_diff);

  //   // Pref = phi_pid.update(TOL_x_err);
  //   // Qref = theta_pid.update(TOL_y_err);
  //   if(auto_mode_count == 0){
  //     auto_mode_count = 1;
  //     // flying_mode = 1;
  //     ideal = mu_Yn_est(1,0);
  //     T_stick = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
  //   }
  //   if (count_up == 10){
  //     input = alt_PID(ideal);
  //     T_ref = T_stick + (input);
  //     printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
  //     // printf("%9.6f %9.6f %9.6f %9.6f %9.6f f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input);
  //     // Hovering();
  //     count_up = 0;
  //   }
  //   count_up = count_up + 1;
  // }

  // //高度制御テスト用のコード
  // if(Chdata[SERVO] > 500){
  //   auto_mode =1;
  // }
  // else{
  //   auto_mode =0;
  //   auto_mode_count = 0;
  //   T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  // }

  // if (auto_mode ==1){
  //   if (count_up >= 10){
  //     count_up = 0;
  //     if(auto_mode_count ==0){
  //       auto_mode_count = 1;
  //       flying_mode = 1;
  //       ideal = Kalman_alt;
  //       T_stick = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  //     }
  //     //printf("Auto Kalman_alt : %9.6f\n",Kalman_alt);
  //     //Auto_fly();
  //     //Auto_takeoff();
  //     //Auto_landing();
  //     Hovering();
  //   }
  //   count_up += 1;
  // }

  // Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  // PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  // saturation P_com
  if (P_com >= 3.7)
  {
    P_com = 3.7;
  }
  else if (P_com <= -3.7)
  {
    P_com = -3.7;
  }

  // saturation R_com
  if (R_com >= 3.7)
  {
    R_com = 3.7;
  }
  else if (R_com <= -3.7)
  {
    R_com = -3.7;
  }

  // saturation Q_com
  if (Q_com >= 3.7)
  {
    Q_com = 3.7;
  }
  else if (Q_com <= -3.7)
  {
    Q_com = -3.7;
  }

  // Motor Control(mixing)


  // printf("%f %f %f %f  \n" ,T_ref,p_ref,q_ref,r_ref  );



  // float FR_duty = (T_ref + (-0.85 *P_com + 1.5 *Q_com - 0.85 *R_com)  ) ;
  // float MR_duty = (T_ref + (-1.75*P_com  + 0.0 *Q_com + 0.85 *R_com)  );
  // float RR_duty = (T_ref + (-0.85*P_com  - 1.5 *Q_com - 0.85 *R_com)  ) ;
  // float RL_duty = (T_ref + ( 0.85*P_com  - 1.5 *Q_com + 0.85 *R_com)  ) ;
  // float ML_duty = (T_ref + ( 1.75*P_com  - 0.0 *Q_com - 0.85 *R_com)  ) ;
  // float FL_duty = (T_ref + ( 0.85 *P_com + 1.5 *Q_com + 0.85 *R_com)  ) ;

  float FR_duty = (T_ref + (-1.7655 *P_com + 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  float MR_duty = (T_ref + (-3.5311*P_com  + 0.0*Q_com     + 1.7655 *R_com)  ) ;
  float RR_duty = (T_ref + (-1.7655*P_com  - 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  float RL_duty = (T_ref + (1.7655*P_com   - 3.0562 *Q_com + 1.7655 *R_com)  ) ;
  float ML_duty = (T_ref + (3.5311*P_com   - 0.0*Q_com     - 1.7655 *R_com)  ) ;
  float FL_duty = (T_ref + (1.7655 *P_com  + 3.0562 *Q_com + 1.7655 *R_com)  ) ;


  float minimum_duty = 0.1;
  const float maximum_duty = 0.95;
  minimum_duty = Disable_duty;

  if (FR_duty < minimum_duty)
    FR_duty = minimum_duty;
  if (FR_duty > maximum_duty)
    FR_duty = maximum_duty;

  if (FL_duty < minimum_duty)
    FL_duty = minimum_duty;
  if (FL_duty > maximum_duty)
    FL_duty = maximum_duty;

  if (MR_duty < minimum_duty)
    MR_duty = minimum_duty;
  if (MR_duty > maximum_duty)
    MR_duty = maximum_duty;

  if (ML_duty < minimum_duty)
    ML_duty = minimum_duty;
  if (ML_duty > maximum_duty)
    ML_duty = maximum_duty;

  if (RR_duty < minimum_duty)
    RR_duty = minimum_duty;
  if (RR_duty > maximum_duty)
    RR_duty = maximum_duty;

  if (RL_duty < minimum_duty)
    RL_duty = minimum_duty;
  if (RL_duty > maximum_duty)
    RL_duty = maximum_duty;

  // Duty set
  if (T_ref  < Disable_duty * BATTERY_VOLTAGE_BASE/BATTERY_VOLTAGE_REAl)
  {
    motor_stop();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();

    Pref = 0.0;
    Qref = 0.0;
    Rref = 0.0;
    Aileron_center = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center = Chdata[0];
    Phi_bias = Phi;
    Theta_bias = Theta;
    Psi_bias = Psi;
  }
  else
  {
    if (OverG_flag == 0)
    {
      // set_duty_fr(FR_duty);//FR_duty
      // set_duty_fl(FL_duty);//FL_duty
      // set_duty_mr(MR_duty);//MR_duty
      // set_duty_ml(ML_duty);//ML_duty
      // set_duty_rr(RR_duty);//RR_duty
      // set_duty_rl(RL_duty);//RL_duty
       
      set_duty_fr(FR_duty);//FR_duty
      set_duty_fl(FL_duty);//MR_duty
      set_duty_mr(MR_duty);//FL_duty
      set_duty_ml(ML_duty);//RL_duty
      set_duty_rr(RR_duty);//FR_duty
      set_duty_rl(RL_duty);//ML_duty
     }

    else
      motor_stop();
    // printf("%12.5f %12.5f %12.5f\n",p_rate, q_rate, r_rate);
  }

  // printf("\n");

  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, fr_duty, fl_duty, rr_duty, rl_duty, p_rate, q_rate, r_rate);
  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, p_com, q_com, r_com, p_ref, q_ref, r_ref);
  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, Phi, Theta, Psi, Phi_bias, Theta_bias, Psi_bias);
  // Elapsed_time = Elapsed_time + 0.0025;
  // Logging
  // logging();
}

extern PMW3901* flow;


void angle_control(void)
{
  float phi_err, theta_err, psi_err;
  float q0, q1, q2, q3;
  float e23, e33, e13, e11, e12;
  while (1)
  {
    sem_acquire_blocking(&sem); // 時間統制
    sem_reset(&sem, 0);
    S_time2 = time_us_32();
    kalman_filter(); // 100Hzで計算
    q0 = Xe(0, 0);
    q1 = Xe(1, 0);
    q2 = Xe(2, 0);
    q3 = Xe(3, 0);
    e11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    e12 = 2 * (q1 * q2 + q0 * q3);
    e13 = 2 * (q1 * q3 - q0 * q2);
    e23 = 2 * (q2 * q3 + q0 * q1);
    e33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23 * e23 + e33 * e33));
    Psi = atan2(e12, e11);
    Psi = Xn_est_3;

    // Get angle ref (manual flight)
    if (1)
    {
      // // Rockingwing
      // if (Flight_mode != ROCKING)
      // {
      //   // Phi_ref   =  Phi_trim + 0.3 *M_PI*(float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5)*2/(CH4MAX-CH4MIN);
      // }
      // else if (Flight_mode == ROCKING)
      // {
      //   Phi_ref = rocking_wings(Phi_ref);
      // }

      // if (Flight_mode == LINETRACE && i2c_connect == 1)
      if (Flight_mode == 100 && i2c_connect == 1)
      {
        // // auto_mode_count = 1;
        // psi_pid.set_parameter(10.0, 1000, 0.01, 0.125, 0.01);
        // if (Linetrace_counter_for_control > 3)
        // {
        //   linetrace();
        //   Linetrace_counter_for_control = 0;
        // }
        // Linetrace_counter_for_control++;

        // if (auto_mode_count == 1) {
        //   auto_mode_count = 0;
        //   ideal = Kalman_alt;
        //   T_stick = 0.6 * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
        // }
        // Hovering();
      }
      else
      {
        if (Flight_mode != ROCKING)
        // if (Flight_mode != 200)
        {
          Phi_trim =-0.6*M_PI/180.0; ;//-14*M_PI/180.0;
          Phi_ref = Phi_trim + 0.8 * M_PI * (float)(Chdata[3] - (CH4MAX + CH4MIN) * 0.5) * 2 / (CH4MAX - CH4MIN);
          //printf("phi_ref,%f\r\n",Phi_ref);
          // printf("1 : %f\r\n",(float)(Chdata[3] - (CH4MAX + CH4MIN) * 0.5) * 2 / (CH4MAX - CH4MIN));
          // printf("2 : %f\r\n",(float)(Chdata[3] - (CH4MAX + CH4MIN))  / (CH4MAX - CH4MIN));
        }

        Theta_trim = 4*M_PI/180;//-10*M_PI/180.0;
        Theta_ref = Theta_trim + 0.5 * M_PI * (float)(Chdata[1] - (CH2MAX + CH2MIN) * 0.5) * 2 / (CH2MAX - CH2MIN);
        Psi_ref = 0.8 * M_PI * (float)(Chdata[0] - (CH1MAX + CH1MIN) * 0.5) * 2 / (CH1MAX - CH1MIN);
        Psi = 0.0;
        // Linetrace_counter = 0;
        psi_pid.set_parameter(0, 100000, 0.01, 0.125, 0.01);
        // auto_mode_count = 1;
        Range_filter.reset();
        Angle_filter.reset();
        Velocity_filter.reset();
      }
    }

    // PID Control
    if (T_ref  < Flight_duty*BATTERY_VOLTAGE_BASE/BATTERY_VOLTAGE_REAl)
    {
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
      Aileron_center = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center = Chdata[0];
      /////////////////////////////////////
      Phi_bias = Phi;
      Theta_bias = Theta;
      Psi_bias = Psi;
      /////////////////////////////////////
    }
    else
    {
      phi_err = Phi_ref - (Phi - Phi_bias);
      theta_err = Theta_ref - (Theta - Theta_bias);
      psi_err = Psi_ref - (Psi - Psi_bias);

      // Pref = Phi_ref;//phi_pid.update(phi_err);
      // Qref = Theta_ref;//theta_pid.update(theta_err);
      // Rref = Psi_ref; // psi_pid.update(psi_err);
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = psi_pid.update(psi_err);
      if (Flight_mode != LINETRACE)
      {
        Rref = Psi_ref;
      }
      // else if (Flight_mode == LINETRACE)
      else if (Flight_mode == 100)
      {
        Rref = psi_pid.update(psi_err);
      }
    }

    // saturation Rref
    if (Rref >= (rate_limit * pi / 180))
    {
      Rref = rate_limit * pi / 180;
    }
    else if (Rref <= -(rate_limit * pi / 180))
    {
      Rref = -(rate_limit * pi / 180);
    }

    // saturation Pref
    if (Pref >= (rate_limit * pi / 180))
    {
      Pref = rate_limit * pi / 180;
    }
    else if (Pref <= -(rate_limit * pi / 180))
    {
      Pref = -(rate_limit * pi / 180);
    }

    // saturation Qref
    if (Qref >= (rate_limit * pi / 180))
    {
      Qref = rate_limit * pi / 180;
    }
    else if (Qref <= -(rate_limit * pi / 180))
    {
      Qref = -(rate_limit * pi / 180);
    }

    // Logging  100Hzで情報を記憶
    logging();

    E_time2 = time_us_32();
    D_time2 = E_time2 - S_time2;
    
    flow_sensor();

  }
}

// --------------------------------Rocking wings---------------------------------------
float rocking_wings(float stick)
{
  float angle = 20; //[deg]
  float f = 5.0;    //[Hz]

  if (Rocking_timer < 2.0)
  {
    Rocking_timer = Rocking_timer + 0.01;
    // rgbled_rocking();
    return angle * M_PI / 180 * sin(f * 2 * M_PI * Rocking_timer);
  }
  else
  {
    rgbled_normal();
    return stick;
  }
}

void logging(void)
{
  // Logging
  //  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5)
  if (Chdata[LOG] > 200)
  {
    if (Logflag == 0)
    {
      Logflag = 1;
      LogdataCounter = 0;
    }
    if (LogdataCounter + DATANUM < LOGDATANUM)
    {
      Logdata[LogdataCounter++] = Xe(0, 0);   // 1
      Logdata[LogdataCounter++] = Xe(1, 0);   // 2
      Logdata[LogdataCounter++] = Xe(2, 0);   // 3
      Logdata[LogdataCounter++] = Xe(3, 0);   // 4
      Logdata[LogdataCounter++] = Xe(4, 0);   // 5
      Logdata[LogdataCounter++] = Xe(5, 0);   // 6
      Logdata[LogdataCounter++] = Xe(6, 0);   // 7
      Logdata[LogdataCounter++] = Wp - Pbias; // 8
      Logdata[LogdataCounter++] = Wq - Qbias; // 9
      Logdata[LogdataCounter++] = Wr - Rbias; // 10

      Logdata[LogdataCounter++] = Ax;                 // 11
      Logdata[LogdataCounter++] = Ay;                 // 12
      Logdata[LogdataCounter++] = Az;                 // 13
      Logdata[LogdataCounter++] = Mx;                 // 14
      Logdata[LogdataCounter++] = My;                 // 15
      Logdata[LogdataCounter++] = Mz;                 // 16
      Logdata[LogdataCounter++] = Pref;               // 17
      Logdata[LogdataCounter++] = Qref;               // 18
      Logdata[LogdataCounter++] = Rref;               // 19
      Logdata[LogdataCounter++] = Phi - Phi_bias;     // 20
      Logdata[LogdataCounter++] = Theta - Theta_bias; // 21
      Logdata[LogdataCounter++] = Psi - Psi_bias;     // 22
      Logdata[LogdataCounter++] = Phi_ref;            // 23
      Logdata[LogdataCounter++] = Theta_ref;          // 24
      Logdata[LogdataCounter++] = Psi_ref;            // 25
      Logdata[LogdataCounter++] = P_com;              // 26
      Logdata[LogdataCounter++] = Q_com;              // 27
      Logdata[LogdataCounter++] = R_com;              // 28

      Logdata[LogdataCounter++] = altitude; // m_filter_output;    //29
      Logdata[LogdataCounter++] = ideal; // m_filter_output;    //30
      Logdata[LogdataCounter++] = T_ref; // m_filter_output;    //31

      Logdata[LogdataCounter++] = T_stick;   // m_filter_output;  //32
      Logdata[LogdataCounter++] = input; // m_filter_output;//33
    }
    else
      Logflag = 2;
  }
  else
  {
    if (Logflag > 0)
    {
      Logflag = 0;
      LogdataCounter = 0;
    }
  }
}

void log_output(void)
{
  if (LogdataCounter == 0)
  {
    printPQR();
    printf("#Roll rate PID gain\n");
    p_pid.printGain();
    printf("#Pitch rate PID gain\n");
    q_pid.printGain();
    printf("#Yaw rate PID gain\n");
    r_pid.printGain();
    printf("#Roll angle PID gain\n");
    phi_pid.printGain();
    printf("#Pitch angle PID gain\n");
    theta_pid.printGain();
  }
  if (LogdataCounter + DATANUM < LOGDATANUM)
  {
    // LockMode=0;
    printf("%10.2f ", Log_time);
    Log_time = Log_time + 0.01;
    for (uint8_t i = 0; i < DATANUM; i++)
    {
      printf("%12.5f", Logdata[LogdataCounter + i]);
    }
    printf("\n");
    LogdataCounter = LogdataCounter + DATANUM;
  }
  else
  {
    Arm_flag = 3;
    Logoutputflag = 0;
    LockMode = 0;
    Log_time = 0.0;
    LogdataCounter = 0;
  }
}

void gyroCalibration(void)
{
  float wp, wq, wr;
  float sump, sumq, sumr;
  uint16_t N = 400;
  for (uint16_t i = 0; i < N; i++)
  {
    sensor_read();
    sump = sump + Wp;
    sumq = sumq + Wq;
    sumr = sumr + Wr;
  }
  Pbias = sump / N;
  Qbias = sumq / N;
  Rbias = sumr / N;
}

// OpenMV通信用----------------------------------------------------------------------------------------------------------------------------
//  void processReceiveData(){
//
//    //   //姿勢によるライン検知の誤差補正
//    //   x_diff = -x_diff;
//    //   angle_diff = -angle_diff*M_PI/180.0;
//    //   x_alpha = atan2(x_diff,118);
//    //   x_diff_dash = 700*tan(Phi + x_alpha); //角度補正

//     //座標変換----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     // float e11,e12,e13,e21,e22,e23,e31,e32,e33;//透視変換の内部パラメータ
//     // float E11,E12,E13,E21,E22,E23,E31,E32,E33;//方向余弦行列
//     // float fx,fy,cx,cy,u1_camera,v1_camera,u2_camera,v2_camera,u1_camera_dash,u2_camera_dash,v1_camera_dash,v2_camera_dash,x1_camera,x2_camera,y1_camera,y2_camera,z1_camera,z2_camera;//透視変換
//     // float x1_drone,x2_drone,y1_drone,y2_drone,z1_drone,z2_drone;//カメラ座標から機体座標へ座標変換
//     // float X1_inertia,Y1_inertia,Z1_inertia,X2_inertia,Y2_inertia,Z2_inertia,X0_1_inertia,Y0_1_inertia,Z0_1_inertia,X0_2_inertia,Y0_2_inertia,Z0_2_inertia;//機体座標から慣性座標に座標変換

//     // //画像カメラからカメラ座標への座標変換
//     // u1_camera = x_1_dash;
//     // v1_camera = y_1_dash;
//     // u2_camera = x_2_dash;
//     // v2_camera = y_2_dash;

//     // //焦点距離は2.8mm
//     // //受光素子数が横：120,縦：120
//     // //ピクセルサイズ：2.8㎛×2.8㎛

//     // //画像座標、カメラ座標の原点を中心に移動
//     // u1_camera_dash = 0.0028*u1_camera + 0.0014 - (0.0028*160)/2;
//     // u2_camera_dash = 0.0028*u2_camera + 0.0014 - (0.0028*160)/2;
//     // v1_camera_dash = 0.0028*v1_camera + 0.0014 - (0.0028*120)/2;
//     // v2_camera_dash = 0.0028*v2_camera + 0.0014 - (0.0028*120)/2;

//     // //イメージセンサの長さ=ピクセルサイズ*受光素子数
//     // fx = (160/0.336)*2.8;//(横の受光素子数/イメージセンサの横の長さ)*mm単位の焦点距離
//     // fy = (120/0.448)*2.8;//(縦の受光素子数/イメージセンサの縦の長さ)*mm単位の焦点距離

//     // cx = 80.5*0.0028;//x軸方向の光学中心、（横のピクセル数*ピクセルサイズ）/2
//     // cy = 60.5*0.0028;//y軸方向の光学中心、（縦のピクセル数*ピクセルサイズ）/2

//     // //透視変換の内部パラメータ（画像座標→カメラ座標）
//     // e11 = 1/fx;
//     // e12 = 0;
//     // e13 = -cx/fy;
//     // e21 = 0;
//     // e22 = 1/fy;
//     // e23 = -cy/fy;
//     // e31 = 0;
//     // e32 = 0;
//     // e33 = 1;

//     // //透視変換(画像座標→カメラ座標)
//     // x1_camera = Kalman_alt*e11*u1_camera_dash;
//     // y1_camera = Kalman_alt*e22*v1_camera_dash;
//     // x2_camera = Kalman_alt*e11*u2_camera_dash;
//     // y2_camera = Kalman_alt*e22*v2_camera_dash;
//     // z1_camera = Kalman_alt*(e31*u1_camera + e32*v1_camera + 1);
//     // z2_camera = Kalman_alt*(e31*u2_camera + e32*v2_camera + 1);

//     // //カメラ座標から機体座標への座標変換
//     // y1_drone = x1_camera;
//     // x1_drone = -y1_camera;
//     // y2_drone = x2_camera;
//     // x2_drone = -y2_camera;
//     // z1_drone = z1_camera;
//     // z2_drone = z2_camera;

//     // //クォータニオンの計算
//     // q0 = Xe(0,0);
//     // q1 = Xe(1,0);
//     // q2 = Xe(2,0);
//     // q3 = Xe(3,0);

//     // //転置後の方向余弦行列（機体座標→慣性座標）
//     // E11 = q0^2 + q1^2 + q2^2 + q3^2;
//     // E12 = 2*(q1*q2 - q0*q3);
//     // E13 = 2*(q1*q3 - q0*q3);
//     // E21 = 2*(q1*q2 - q0*q3);
//     // E22 = q0^2 - q1^2 + q2^2 - q3^2;
//     // E23 = 2*(q1*q3 - q0*q1);
//     // E31 = 2*(q1*q3 - q0*q2);
//     // E32 = 2*(q2*q3 + q0*q1);
//     // E33 = q0^2 - q1^2 - q2^2 + q3^2;

//     // //機体座標から慣性座標への座標変換
//     // X1_inertia = E11*x1_drone + E12*y1_drone + E13*z1_drone;
//     // Y1_inertia = E21*x1_drone + E22*y1_drone + E23*z1_drone;
//     // Z1_inertia = E31*x1_drone + E32*y1_drone + E33*z1_drone;

//     // X2_inertia = E11*x2_drone + E12*y2_drone + E13*z2_drone;
//     // Y2_inertia = E21*x2_drone + E22*y2_drone + E23*z2_drone;
//     // Z2_inertia = E31*x2_drone + E32*y2_drone + E33*z2_drone;

//     // //カメラから得られた2点の画像座標を慣性座標上のドローンの位置の座標
//     // X0_1_inertia = 0 - X1_inertia;
//     // Y0_1_inertia = 0 - Y1_inertia;
//     // Z0_1_inertia = 0 - Z1_inertia;

//     // X0_2_inertia = 0 - X2_inertia;
//     // Y0_2_inertia = 0 - Y2_inertia;
//     // Z0_2_inertia = 0 - Z2_inertia;

//     // //慣性空間に変換したドローン自身の2点の座標から直線の方程式を出す
//     // float Inclination,b;

//     // Inclination = (Y0_2_inertia - Y0_1_inertia)/(X0_2_inertia - X0_1_inertia);//直線の方程式の傾き
//     // b = ((X0_2_inertia*Y0_1_inertia)/X0_1_inertia - Y0_2_inertia)/(X0_2_inertia/X0_1_inertia - 1);//直線の方程式の切片

//     // Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
//     // Line_velocity = Velocity_filter.update(Xn_est_1); //速度
//     // Line_range = Range_filter.update(Xn_est_2); //横ずれ
//     // Psi = Angle_filter.update(Xn_est_3); //ラインとの角度

//     // // current_time = time_us_64();
//     // //printf("x : %9.6f\n",x_diff);
//     // printf("red_circle : %f\n",red_circle);
//     // printf("length : %f\n",length);
// }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// }

//OpenMV通信用
void processReceiveData(){

  char* clear_data = buffer;
  clear_data++;//(をスキップ
  clear_data[strlen(clear_data) -1 ] = '\0';//)をヌル文字に置き換え
  char* token;

  // if (Flight_mode == HOVERING)
  // {
  //   token = strtok(clear_data,",");
  //   if (token != NULL){
  //     TOL_x_diff = atof(token);
  //   }
  //   token = strtok(NULL,",");
  //   if (token != NULL){
  //     TOL_y_diff = atof(token);
  //   }
  //   // printf("%9.6f \n",TOL_x_diff);
  //   // printf("%9.6f \n",TOL_y_diff);
  // }

  // if (Flight_mode == LINETRACE)
  if (Flight_mode == 100)
  {
     token = strtok(clear_data,",");
      if (token != NULL){
        x_diff = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        angle_diff = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        gap_number = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        line_number = atof(token);
      }
      //x_diff_dash = ((2 * 500 * tan(35) * -x_diff) / 160) - (500 * tan(Phi));
      // x_diff_dash = ((x_diff * 497.46) / 80) - (700*tan(Phi));
      x_alpha = atan2(x_diff,118);
      x_diff_dash = 700 * tan(Phi - x_alpha);
      Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
      Line_range = Xn_est_2; //横ずれ
      Line_velocity = Xn_est_1; //速度
      // current_time = time_us_64();
      // printf("x_diff : %9.6f angle_diff : %9.6f gap : %9.6f  line_num : %9.6f\n",x_diff,angle_diff,gap_number,line_number);


    // token = strtok(clear_data,",");
    // if (token != NULL){
    //   TOL_x_diff = atof(token);
    // }
    // token = strtok(NULL,",");
    // if (token != NULL){
    //   TOL_y_diff = atof(token);
    // }
    // printf("%9.6f \n",TOL_x_diff);
    // printf("%9.6f \n",TOL_y_diff);

    // if(landing_counter ==1){
    //   token = strtok(clear_data,",");
    //   if (token != NULL){
    //     TOL_x_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     TOL_y_diff = atof(token);
    //   }
    //   // TOL_x_alpha = atan2(TOL_x_diff,118);
    //   // TOL_y_alpha = atan2(TOL_y_diff,87);
    //   // TOL_x_diff_dash = 700 * tan(Phi - TOL_x_alpha);
    //   // TOL_y_diff_dash = 700 * tan(Theta - TOL_y_alpha);
    //   // printf("%9.6f %9.6f\n",TOL_x_diff,TOL_x_diff_dash);
    //   // printf("%9.6f %9.6f\n",TOL_y_diff,TOL_y_diff_dash);
    // }
    // else{
    //   token = strtok(clear_data,",");
    //   if (token != NULL){
    //     x_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     angle_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     gap_number = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     line_number = atof(token);
    //   }
    //   //x_diff_dash = ((2 * 500 * tan(35) * -x_diff) / 160) - (500 * tan(Phi));
    //   // x_diff_dash = ((x_diff * 497.46) / 80) - (700*tan(Phi));
    //   x_alpha = atan2(x_diff,118);
    //   x_diff_dash = 700 * tan(Phi - x_alpha);
    //   Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
    //   Line_range = Xn_est_2; //横ずれ
    //   Line_velocity = Xn_est_1; //速度
    //   // current_time = time_us_64();
    //   printf("x_diff : %9.6f angle_diff : %9.6f gap : %9.6f  line_num : %9.6f\n",x_diff,angle_diff,gap_number,line_number);
    //   //printf("x : %9.6f Est : %9.6f dash : %9.6f\n",x_diff,Line_range,x_diff_dash);
    //   // printf("angle : %9.6f est : %9.6f\n",angle_diff,Xn_est_3);
    //   //printf("%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f\n",(current_time - start_time )/1000000,x_diff,Line_range,x_diff_dash,Theta,Phi,angle_diff,Xn_est_3,Xn_est_1,x_alpha);
    // }
  }

  else if (Flight_mode == REDCIRCLE){
    // token = strtok(clear_data,",");
    // if (token != NULL){
    //   red_circle = atof(token);
    // }
    red_circle = atof(clear_data);
    // printf("red_circle: %9.6f\n", red_circle);
    //printf("KAWASAKI");
  }

  // printf("x : %9.6f\n",x_diff);
  // printf("angle : %9.6f\n",angle_diff);
  // printf("est velocity: %9.6f\n",Xn_est_1);
  // printf("y : %9.6f, est : %9.6f\n",x_diff,Xn_est_2);
  // printf("psi : %9.6f, est : %9.6f\n",angle_diff,Xn_est_3);
}

void receiveData(char c){
  if (buffer_index < BUFFER_SIZE - 1){
    buffer[buffer_index++] = c;
  }
  //終了条件のチェック
  // if (c == '\n'){
  if (c == ')'){
    //buffer[buffer_index] = '\0'; //文字列の終端にヌル文字を追加
    processReceiveData();
    buffer_index = 0; //バッファをリセット
  }
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax = -acceleration_mg[0] * GRAV * 0.001;
  Ay = -acceleration_mg[1] * GRAV * 0.001;
  Az = acceleration_mg[2] * GRAV * 0.001;
  Wp = angular_rate_mdps[0] * M_PI * 5.55555555e-6; // 5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1] * M_PI * 5.55555555e-6;
  Wr = -angular_rate_mdps[2] * M_PI * 5.55555555e-6;
  Mx0 = -magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 = -magnetic_field_mgauss[2];

  // 加速度・角速度のリミッター
  acc_norm = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  if (acc_norm > 800.0)
    OverG_flag = 1;
  // Acc_norm = acc_filter.update(acc_norm);
  // rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  // if (rate_norm > 70.0) OverG_flag =1;

  /*地磁気校正データ
  回転行列
  [[ 0.65330968  0.75327755 -0.07589064]
  [-0.75666134  0.65302622 -0.03194321]
  [ 0.02549647  0.07829232  0.99660436]]
  中心座標
  122.37559195017053 149.0184454603531 -138.99116060635413
  W
  -2.432054387460946
  拡大係数
  0.003077277151877191 0.0031893151610213463 0.0033832794976645804

  //回転行列
  const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                    -0.75666134, 0.65302622, -0.03194321,
                      0.02549647, 0.07829232,  0.99660436};
  //中心座標
  const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
  //拡大係数
  const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};
  */
  // 回転行列
  const float rot[9] = {-0.78435472, -0.62015392, -0.01402787,
                        0.61753358, -0.78277935, 0.07686857,
                        -0.05865107, 0.05162955, 0.99694255};
  // 中心座標
  const float center[3] = {-109.32529343620176, 72.76584808916506, 759.2285249891385};
  // 拡大係数
  const float zoom[3] = {0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

  // 回転・平行移動・拡大
  mx1 = zoom[0] * (rot[0] * Mx0 + rot[1] * My0 + rot[2] * Mz0 - center[0]);
  my1 = zoom[1] * (rot[3] * Mx0 + rot[4] * My0 + rot[5] * Mz0 - center[1]);
  mz1 = zoom[2] * (rot[6] * Mx0 + rot[7] * My0 + rot[8] * Mz0 - center[2]);
  // 逆回転
  Mx = rot[0] * mx1 + rot[3] * my1 + rot[6] * mz1;
  My = rot[1] * mx1 + rot[4] * my1 + rot[7] * mz1;
  Mz = rot[2] * mx1 + rot[5] * my1 + rot[8] * mz1;

  mag_norm = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx /= mag_norm;
  My /= mag_norm;
  Mz /= mag_norm;

// 高度センサーから値受け取るコード（new）
  // altitude_count = altitude_count + 1;
  // if (altitude_count == 8) { // 400Hzを8回に1回実行 = 50Hzで更新
  //   altitude_count = 0;
    
  //   uint16_t z_mm = 0;
  //   // ToFセンサーから新しい距離データが取得できた場合
  //   if (tof_read_valid(&z_mm)) {
  //     float distance = (float)z_mm; // ToFの生の値（ミリメートル）
      
  //     z_acc = Az - 9.76548; // Z軸加速度から重力加速度成分を引く
      
  //     // ドローンの傾きから、真下方向の距離を計算（三角関数による補正）
  //     lotate_altitude_init(Theta, Psi, Phi);
  //     lotated_distance = lotate_altitude(distance);
      
  //     // カルマンフィルタに補正済み距離と加速度を渡して、滑らかな高度を推定
  //     Kalman_alt = Kalman_PID(lotated_distance, z_acc);
  //     altitude = mu_Yn_est(1,0); 
      
  //     // シリアルモニタでの確認用（約1秒に1回出力）
  //     static uint32_t print_count = 0;
  //     if (print_count++ > 50) { 
  //         printf("Raw: %4.0f mm | Corrected: %4.1f mm | Kalman: %4.1f mm\r\n", distance, lotated_distance, Kalman_alt);
  //         print_count = 0;
  //     }
  //   }
      altitude_count = altitude_count + 1;
    if (altitude_count == 8){
      altitude_count = 0;
      uint8_t checkdata[2];
      int result = i2c_read_blocking(I2C_PORT, dev, checkdata, sizeof(checkdata), false);
      if (result != 2)
      {
        // I2C通信エラーチェック
        // エラーが発生した場合、I2C通信が切断されたとみなす
        // ここで適切なエラーハンドリングを行う
        Flight_mode = NORMAL;
        i2c_connect = 0;
        printf("I2C通信エラーが発生しました。 %4d\n", result);
      }
      else
      {
        // printf("I2C通信接続できました。 %4d\n",result);
        // 高度センサーから値受け取るコード
        if (isDataReady == 0)
        {
          Status = VL53L1X_CheckForDataReady(dev, &isDataReady);
        }
        else if (isDataReady == 1)
        {
          isDataReady = 0;
          Status = VL53L1X_GetRangeStatus(dev, &rangeStatus);
          Status = VL53L1X_GetDistance(dev, &distance);
          Status = VL53L1X_ClearInterrupt(dev);
          // z_acc  = Az-9.80665;
          z_acc = Az - 9.76548;
          lotate_altitude_init(Theta, Psi, Phi);
          lotated_distance = lotate_altitude(distance);
          Kalman_alt = Kalman_PID(lotated_distance, z_acc);
          altitude = mu_Yn_est(1,0);

          static int print_counter = 0;
          print_counter++;
          if(Flight_mode !=HOVERING){

            if (print_counter >= 50) { // 50Hz → 1Hz
              print_counter = 0;

              printf("Raw: %d mm\n", distance);                 // 生値
              printf("Corrected: %f mm\n", lotated_distance);   // 姿勢補正
              printf("Kalman: %f mm\n\n", Kalman_alt);          // フィルタ
            }
          }

        }
      }
    }

  }

void variable_init(void)
{
  // Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Xp = Xe;

  Q << 6.0e-5, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 5.0e-5, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.8e-5, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 5.0e-5, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 5.0e-5, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 5.0e-5;

  R << 1.701e0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 2.799e0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.056e0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 2.3e-1, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.4e-1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.49e-1;

  G << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
      -1.0, 1.0, -1.0, 0.0, 0.0, 0.0,
      -1.0, -1.0, 1.0, 0.0, 0.0, 0.0,
      1.0, -1.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Beta << 0.0, 0.0, 0.0;

  P << 1e0, 0, 0, 0, 0, 0, 0,
      0, 1e0, 0, 0, 0, 0, 0,
      0, 0, 1e0, 0, 0, 0, 0,
      0, 0, 0, 1e0, 0, 0, 0,
      0, 0, 0, 0, 1e0, 0, 0,
      0, 0, 0, 0, 0, 1e0, 0,
      0, 0, 0, 0, 0, 0, 1e0;
}

void printPQR(void)
{
  volatile int m = 0;
  volatile int n = 0;
  // Print P
  printf("#P\n");
  for (m = 0; m < 7; m++)
  {
    printf("# ");
    for (n = 0; n < 7; n++)
    {
      printf("%12.4e ", P(m, n));
    }
    printf("\n");
  }
  // Print Q
  printf("#Q\n");
  for (m = 0; m < 6; m++)
  {
    printf("# ");
    for (n = 0; n < 6; n++)
    {
      printf("%12.4e ", Q(m, n));
    }
    printf("\n");
  }
  // Print R
  printf("#R\n");
  for (m = 0; m < 6; m++)
  {
    printf("# ");
    for (n = 0; n < 6; n++)
    {
      printf("%12.4e ", R(m, n));
    }
    printf("\n");
  }
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n",
         Elapsed_time // 1
         ,
         Xe(0, 0), Xe(1, 0), Xe(2, 0), Xe(3, 0) // 2~5
         ,
         Xe(4, 0), Xe(5, 0), Xe(6, 0) // 6~8
         //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
         ,
         D_time, D_time2 // 10,11
         ,
         Ax, Ay, Az // 11~13
         ,
         Wp, Wq, Wr // 14~16
         ,
         Mx, My, Mz // 17~19
         //,mag_norm
  ); // 20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n",
         Elapsed_time // 1
         ,
         Ax, Ay, Az // 2~4
         ,
         Wp, Wq, Wr // 5~7
         ,
         Mx, My, Mz // 8~10
  );                // 20
}

void kalman_filter(void)
{
  // Kalman Filter
  
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G * dt, Beta, dt);
}

void flow_sensor(void){


    int16_t dx, dy;
    float vx, vy;

    if (flow->readMotionBurst(&dx, &dy)) {

        flow->pmw3901_calculate_velocity_direct(dx, dy, Kalman_alt, dt, &vx, &vy);

        static int cnt = 0;
        // if (++cnt % 10 == 0) {
        //     printf("vx=%f, vy=%f\n", vx, vy);
        // }
    }
}