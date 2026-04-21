#include "pico_copter.hpp"

// ===== グローバル変数 =====
uint8_t Arm_flag = 0;
uint8_t Red_flag = 0;
semaphore_t sem;

// Optical Flow ポインタ
PMW3901* flow = nullptr;

int main(void)
{
    int start_wait = 5;

    // ===== Pico初期化 =====
    stdio_init_all();
    sleep_ms(2000);   // USBシリアル安定待ち

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("# start setup\r\n");

    // ===== IMU =====
    imu_mag_init();
    printf("# init gyro\r\n");

    // ===== Radio =====
    radio_init();
    printf("# init radio\r\n");

    // ===== 変数初期化 =====
    variable_init();
    printf("# init variable\r\n");

    // ===== 制御初期化 =====
    control_init();

    // ===== RGB LED OFF =====
    rgbled_off();

    // ===== PWM =====
    ESC_calib = 0;
    pwm_init();
    printf("# init pwm\r\n");

    // ===== Optical Flow 初期化 =====
    printf("# init optical flow...\r\n");


    // ===== 起動待ち =====
    while (start_wait)
    {
        printf("# Please wait %d[s]\r", start_wait);
        sleep_ms(1000);
        start_wait--;
    }
    printf("\r\n");

    // ===== マルチコア設定 =====
    sem_init(&sem, 0, 1);
    multicore_launch_core1(angle_control);

    Arm_flag = 1;
    printf("# system armed\r\n");

    // 動的生成
    flow = new PMW3901(spi0, 16, 19, 18, 17);
    flow->pmw_init();

    if (flow != nullptr) { // nullptr チェックのみ
        printf("# optical flow ready\r\n");
    } else {
        printf("Error: Optical flow sensor init failed!\n");
    }

    // ===== メインループ =====
    while (1)
    {
        tight_loop_contents();

        // ログ出力
        while (Logoutputflag == 1)
        {
            log_output();
        }

        // SBUS確認
        printf("Ch0=%d Ch1=%d Ch2=%d Ch3=%d\n",
               Chdata[0], Chdata[1], Chdata[2], Chdata[3]);


        sleep_ms(100);
    }

    // ===== 終了処理 =====
    if (flow) {
        delete flow;
        flow = nullptr;
    }

    return 0;
}