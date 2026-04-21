#include "pmw3901.hpp"
#include <cstring>
#include <cstdio>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

PMW3901::PMW3901(spi_inst_t* spi, uint pin_miso, uint pin_mosi, uint pin_sck, uint pin_cs)
    : _spi(spi),
      _pin_miso(pin_miso),
      _pin_mosi(pin_mosi),
      _pin_sck(pin_sck),
      _pin_cs(pin_cs)
{
}
void PMW3901::cs_low() {
    gpio_put(_pin_cs, 0);
}

void PMW3901::cs_high() {
    gpio_put(_pin_cs, 1);
}

bool PMW3901::initRegisters()
{
    uint8_t read_val;

    printf("Step 6.1: Initial settings\n");

    // Initial settings
    writeRegister(0x7F, 0x00);
    writeRegister(0x55, 0x01);
    writeRegister(0x50, 0x07);
    writeRegister(0x7F, 0x0E);

    // =========================
    // Step 6.2: Verification
    // =========================
    printf("Step 6.2: Verification\n");

    bool verification_ok = false;

    for (int i = 0; i < 3; i++) {
        writeRegister(0x43, 0x10);

    if (readRegister(0x47, &read_val) && read_val == 0x08) {
        verification_ok = true;
        printf("Verification OK (%d)\n", i + 1);
        break;
    }

    printf("Verification NG: 0x%02X\n", read_val);        sleep_ms(1);
    }

    if (!verification_ok) {
        printf("Verification FAILED\n");
        return false;
    }

    // =========================
    // Step 6.3: Conditional write
    // =========================
    printf("Step 6.3\n");
    
    readRegister(0x67, &read_val);

    if (read_val & 0x80) {
        printf("Bit7=1 → write 0x04\n");
        writeRegister(0x48, 0x04);
    } else {
        printf("Bit7=0 → write 0x02\n");
        writeRegister(0x48, 0x02);
    }

    // =========================
    // Step 6.4
    // =========================
    printf("Step 6.4\n");

    writeRegister(0x7F, 0x00);
    writeRegister(0x51, 0x7B);
    writeRegister(0x50, 0x00);
    writeRegister(0x55, 0x00);
    writeRegister(0x7F, 0x0E);

    // =========================
    // Step 6.5: C1/C2
    // =========================
    printf("Step 6.5\n");
    
    readRegister(0x73, &read_val) ;
    printf("0x73 = 0x%02X\n", read_val);
    if (read_val == 0x00) {
        uint8_t c1 ;
        uint8_t c2 ;
        readRegister(0x70 ,&c1);
        readRegister(0x71 ,&c2);
        printf("C1=0x%02X C2=0x%02X\n", c1, c2);

        uint8_t new_c1;
        if (c1 <= 28)
            new_c1 = c1 + 14;
        else
            new_c1 = c1 + 11;

        if (new_c1 > 0x3F)
            new_c1 = 0x3F;

        uint8_t new_c2 = (c2 * 45) / 100;

        printf("newC1=0x%02X newC2=0x%02X\n", new_c1, new_c2);

        writeRegister(0x7F, 0x00);
        writeRegister(0x61, 0xAD);
        writeRegister(0x51, new_c1);
        writeRegister(0x7F, 0x0E);
        writeRegister(0x70, new_c1);
        writeRegister(0x71, new_c2);
    } 
    else {
        printf("Skip C1/C2\n");
    }

    // =========================
    // Step 6.6: Main config
    // =========================
    printf("Step 6.6\n");

    writeRegister(0x7F, 0x00);
    writeRegister(0x61, 0xAD);
    writeRegister(0x7F, 0x03);
    writeRegister(0x40, 0x00);
    writeRegister(0x7F, 0x05);
    writeRegister(0x41, 0xB3);
    writeRegister(0x43, 0xF1);
    writeRegister(0x45, 0x14);
    writeRegister(0x5B, 0x32);
    writeRegister(0x5F, 0x34);
    writeRegister(0x7B, 0x08);
    writeRegister(0x7F, 0x06);
    writeRegister(0x44, 0x1B);
    writeRegister(0x40, 0xBF);
    writeRegister(0x4E, 0x3F);
    writeRegister(0x7F, 0x08);
    writeRegister(0x65, 0x20);
    writeRegister(0x6A, 0x18);
    writeRegister(0x7F, 0x09);
    writeRegister(0x4F, 0xAF);
    writeRegister(0x5F, 0x40);
    writeRegister(0x48, 0x80);
    writeRegister(0x49, 0x80);
    writeRegister(0x57, 0x77);
    writeRegister(0x60, 0x78);
    writeRegister(0x61, 0x78);
    writeRegister(0x62, 0x08);

    sleep_ms(10);

    // =========================
    // Step 6.8: Final config
    // =========================
    printf("Step 6.8\n");

    writeRegister(0x32, 0x44);
    writeRegister(0x7F, 0x07);
    writeRegister(0x63, 0x50);
    writeRegister(0x7F, 0x0A);
    writeRegister(0x45, 0x60);
    writeRegister(0x7F, 0x00);
    writeRegister(0x4D, 0x11);
    writeRegister(0x55, 0x80);
    writeRegister(0x74, 0x1F);
    writeRegister(0x75, 0x1F);
    writeRegister(0x4A, 0x78);
    writeRegister(0x4B, 0x78);
    writeRegister(0x44, 0x08);
    writeRegister(0x45, 0x50);
    writeRegister(0x64, 0xFF);
    writeRegister(0x65, 0x1F);
    writeRegister(0x7F, 0x14);
    writeRegister(0x65, 0x67);
    writeRegister(0x66, 0x08);
    writeRegister(0x63, 0x70);
    writeRegister(0x7F, 0x15);
    writeRegister(0x48, 0x48);
    writeRegister(0x7F, 0x07);
    writeRegister(0x41, 0x0D);
    writeRegister(0x43, 0x14);
    writeRegister(0x4B, 0x0E);
    writeRegister(0x45, 0x0F);
    writeRegister(0x44, 0x42);
    writeRegister(0x4C, 0x80);
    writeRegister(0x7F, 0x10);
    writeRegister(0x5B, 0x02);
    writeRegister(0x7F, 0x07);
    writeRegister(0x40, 0x41);
    writeRegister(0x70, 0x00);

    sleep_ms(10);

    writeRegister(0x40, 0x40);
    writeRegister(0x7F, 0x06);
    writeRegister(0x62, 0xF0);
    writeRegister(0x63, 0x00);
    writeRegister(0x7F, 0x0D);
    writeRegister(0x48, 0xC0);
    writeRegister(0x6F, 0xD5);
    writeRegister(0x7F, 0x00);
    writeRegister(0x5B, 0xA0);
    writeRegister(0x4E, 0xA8);
    writeRegister(0x5A, 0x50);
    writeRegister(0x40, 0x80);

    printf("PMW3901 init complete\n");

    return true;
}

bool PMW3901::readRegister(uint8_t reg, uint8_t *value)
{
    uint8_t addr = (uint8_t)(reg & 0x7F); // MSB=0 for Read [6]
    uint8_t data = 0;

    cs_low();

    // ① アドレス送信
    spi_write_blocking(_spi, &addr, 1);

    // ② tSRAD待ち（超重要）
    sleep_us(35);   // ← ここが今回の本質（最低でも数µs）

    // ③ データ受信
    spi_read_blocking(_spi, 0x00, &data, 1);

    cs_high();

    sleep_us(20);

    *value = data;
    return true;
}

bool PMW3901::writeRegister(uint8_t reg, uint8_t value) {
    // 1. 書き込みを示すためアドレスのMSBを1に設定 [4]
    uint8_t addr = (uint8_t)(reg | 0x80);
    uint8_t tx_data[5] = { addr, value };

    // 2. NCSをLowにしてシリアルポートをアクティブにする [6, 7]
    cs_low();

    // 3. アドレスとデータを送信（計2バイト）[4]
    // ※ 多くのSPIドライバは送信完了後すぐに戻るため、直後の遅延が重要
    spi_write_blocking(_spi, tx_data, 2);

    // 4. 【重要】最後のSCLKからNCSをHighに戻すまで35μs待機 (tSCLK-NCS) [3, 8]
    // これを怠ると、チップがデータを内部レジスタに取り込めません。
    sleep_us(35);

    // 5. NCSをHighにしてトランザクションを終了 [7]
    cs_high();

    // 6. 【重要】次のSPIコマンドを開始するまで45μs待機 (tSWW / tSWR) [1, 2, 9, 10]
    // 連続した書き込みや、その後の読み取りを正常に行うための時間です。
    sleep_us(45);

    return true;
}

bool PMW3901::selectBank(uint8_t bank)
{
    return writeRegister(0x7F, bank);
}
bool PMW3901::reset()
{
    cs_high();
    sleep_ms(1);
    cs_low();

    writeRegister(0x3A, 0x5A);
    sleep_ms(2);

    return true;
}

bool PMW3901::pmw_init()
{
    printf("=== PMW3901 Initialization Start ===\n");

    // =========================
    // Step 1: 電源安定待ち
    // =========================
    printf("Step 1: Power stabilization wait\n");
    sleep_ms(50); // 40ms以上
    printf("  Power stable: OK\n");

    // =========================
    // Step 1.5: CSピン設定 & SPIリセット
    // =========================
    gpio_init(_pin_cs);
    gpio_set_dir(_pin_cs, GPIO_OUT);

    printf("Step 1.5: SPI port reset\n");
    cs_high();
    sleep_ms(1);
    cs_low();
    sleep_ms(1);
    cs_high();
    sleep_ms(1);
    printf("  SPI port reset: OK\n");

    // =========================
    // Step 2: SPI初期化
    // =========================
    printf("Step 2: SPI init\n");

    spi_init(_spi, PMW3901_SPI_CLOCK);

    spi_set_format(_spi,
               8,
               SPI_CPOL_1,
               SPI_CPHA_1,
               SPI_MSB_FIRST);

    gpio_set_function(_pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(_pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(_pin_sck,  GPIO_FUNC_SPI);

    printf("  SPI: OK\n");

    // =========================
    // Step 3: ソフトリセット
    // =========================
    printf("Step 3: Reset\n");

    if (!reset()) {
        printf("  Reset failed\n");
        return false;
    }

    sleep_ms(5);
    printf("  Reset: OK\n");

    // =========================
    // Step 4: Motionレジスタクリア
    // =========================
    printf("Step 4: Clear motion registers\n");

    uint8_t dummy;
    readRegister(0x02, &dummy);
    readRegister(0x03, &dummy);
    readRegister(0x04, &dummy);
    readRegister(0x05, &dummy);
    readRegister(0x06, &dummy);

    printf("  Motion clear: OK\n");

    // =========================
    // Step 5: ID確認
    // =========================
    printf("Step 5: Chip ID check\n");

    uint8_t product_id, inverse_id;

    readRegister(0x00, &product_id);
    readRegister(0x5F, &inverse_id);

    printf("  Product ID: 0x%02X\n", product_id);
    printf("  Inverse ID: 0x%02X\n", inverse_id);

    if (product_id != 0x49) {
        printf("  Invalid Product ID!\n");
        return false;
    }

    if (inverse_id != 0xB6) {
        printf("  Invalid Inverse ID!\n");
        return false;
    }

    printf("  Chip verification: OK\n");

    // =========================
    // Step 6: レジスタ初期化
    // =========================
    printf("Step 6: Register init\n");

    if (!initRegisters()) {
        printf("  Register init failed\n");
        return false;
    }

    printf("  Register init: OK\n");

    // =========================
    // Step 7: 安定待ち
    // =========================
    printf("Step 7: Stabilization\n");
    sleep_ms(100);

    // =========================
    // Step 8: 最終クリア
    // =========================
    printf("Step 8: Final motion clear\n");

    readRegister(0x02, &dummy);
    readRegister(0x03, &dummy);
    readRegister(0x04, &dummy);
    readRegister(0x05, &dummy);
    readRegister(0x06, &dummy);

    printf("  Final clear: OK\n");

    printf("=== PMW3901 Initialization Complete ===\n");

    return true;
}


bool PMW3901::pmw_deinit(){
    
    cs_high();

    // GPIOを通常状態に戻す（任意）
    gpio_set_function(_pin_miso, GPIO_FUNC_NULL);
    gpio_set_function(_pin_mosi, GPIO_FUNC_NULL);
    gpio_set_function(_pin_sck,  GPIO_FUNC_NULL);

    return true;
}

bool PMW3901::readMotion(int16_t *delta_x, int16_t *delta_y) {
    // printf("PMW3901 sTep1\n");

    uint8_t motion, xl, xh, yl, yh;
    uint8_t squal, shutter_upper;
    // printf("PMW3901 sTep2\n");

    // 1. Motionレジスタを読み取り、データをフリーズさせる [1]
if (!readRegister(0x02, &motion))
    return false;

// printf("motion=0x%02X\n", motion);

// ★ここで先に読んでしまう
readRegister(0x07, &squal);
readRegister(0x0C, &shutter_upper);

// printf("squal=%d\n", squal);
// printf("shutter=%d\n", shutter_upper);

// そのあとに判定
if (!(motion & 0x80)) {
    *delta_x = 0;
    *delta_y = 0;
    return true;
}

    // 動きがない場合は0を返して終了（Bit 7を確認）[2]

    // printf("PMW3901 sTep3\n");

    // 2. 移動量レジスタを順番に読み取る [5], [6]
    readRegister(0x03, &xl);
    readRegister(0x04, &xh);
    readRegister(0x05, &yl);
    readRegister(0x06, &yh); // 修正済み

    // 3. 偽モーション抑制のための追加情報を読み取る [3]
    readRegister(0x07, &squal);
    readRegister(0x0C, &shutter_upper);
   
    // printf("PMW3901 sTep4\n");

    // 4. データの検証 [3], [4]
    if (squal < 0x19 && shutter_upper == 0x1F) {
        *delta_x = 0;
        *delta_y = 0;
        return true; // 無効なデータとして処理
    }
    // printf("PMW3901 sTep5\n");

    // 5. 16ビット符号付き整数に結合 [2]
    *delta_x = (int16_t)((xh << 8) | xl);
    *delta_y = (int16_t)((yh << 8) | yl);
    // printf("PMW3901 sTep6\n");

    return true;
}