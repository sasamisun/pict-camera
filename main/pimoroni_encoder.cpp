#include "pimoroni_encoder.h"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===== ログタグ定義 =====
static const char *TAG = "pimoroni_encoder";

// ===== ポーリングタスク設定 =====
#define ENCODER_POLL_INTERVAL_MS 10          // ポーリング間隔
#define ENCODER_DEBOUNCE_SAME_DIR_MS 50      // 同方向デバウンス時間（高速応答）
#define ENCODER_DEBOUNCE_OPPOSITE_DIR_MS 500 // 逆方向デバウンス時間（チャタリング抑制）
#define ACCUMULATOR_TIMEOUT_MS 500           // アキュムレータタイムアウト

// ===== 内部関数プロトタイプ =====
static void encoder_poll_task(void *arg);
static esp_err_t write_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t value);
static esp_err_t read_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t *value);
static esp_err_t read_register_16bit(pimoroni_encoder_t *encoder, uint8_t reg_low, uint8_t reg_high, uint16_t *value);
static esp_err_t set_pin_mode_pwm(pimoroni_encoder_t *encoder, uint8_t physical_pin, uint8_t pwm_channel, bool invert);
static esp_err_t setup_rotary_encoder_hardware(pimoroni_encoder_t *encoder);
static esp_err_t setup_pwm_system(pimoroni_encoder_t *encoder);
static esp_err_t enable_interrupt_output(pimoroni_encoder_t *encoder);

// ===== I2C通信関数 =====

/**
 * @brief レジスタに1バイト書き込み
 */
static esp_err_t write_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};

    esp_err_t ret = i2c_master_write_to_device(
        encoder->i2c_port,
        encoder->i2c_address,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C書き込みエラー reg=0x%02X, value=0x%02X", reg, value);
    }

    return ret;
}

/**
 * @brief レジスタから1バイト読み取り
 */
static esp_err_t read_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t *value)
{
    esp_err_t ret = i2c_master_write_read_device(
        encoder->i2c_port,
        encoder->i2c_address,
        &reg,
        1,
        value,
        1,
        pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C読み取りエラー reg=0x%02X", reg);
    }

    return ret;
}

/**
 * @brief 16ビットレジスタ読み取り（下位→上位の順）
 */
static esp_err_t read_register_16bit(pimoroni_encoder_t *encoder, uint8_t reg_low, uint8_t reg_high, uint16_t *value)
{
    uint8_t low_byte, high_byte;
    esp_err_t ret;

    ret = read_register(encoder, reg_low, &low_byte);
    if (ret != ESP_OK)
        return ret;

    ret = read_register(encoder, reg_high, &high_byte);
    if (ret != ESP_OK)
        return ret;

    *value = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
    return ESP_OK;
}

// ===== ハードウェア設定関数 =====

/**
 * @brief ピンをPWMモードに設定
 */
static esp_err_t set_pin_mode_pwm(pimoroni_encoder_t *encoder, uint8_t physical_pin, uint8_t pwm_channel, bool invert)
{
    esp_err_t ret = ESP_OK;
    
    // ピン番号からポートとビット位置を計算
    uint8_t port, pin_bit;
    
    switch (physical_pin) {
        case 1:  port = 1; pin_bit = 5; break;  // ピン1 → P1.5
        case 2:  port = 1; pin_bit = 0; break;  // ピン2 → P1.0
        case 7:  port = 1; pin_bit = 1; break;  // ピン7 → P1.1
        default:
            ESP_LOGE(TAG, "未対応の物理ピン: %d", physical_pin);
            return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "PWM設定: ピン%d → ポート%d:ビット%d, PWMch%d", 
             physical_pin, port, pin_bit, pwm_channel);
    
    // PWMモード設定用のレジスタアドレス
    uint8_t reg_m1 = (port == 0) ? REG_P0M1 : REG_P1M1;
    uint8_t reg_m2 = (port == 0) ? REG_P0M2 : REG_P1M2;
    
    // 現在のモード設定を読み取り
    uint8_t m1_val, m2_val;
    ret |= read_register(encoder, reg_m1, &m1_val);
    ret |= read_register(encoder, reg_m2, &m2_val);
    
    if (ret != ESP_OK) return ret;
    
    // PWMモード設定（Push-Pull出力: M1=0, M2=1）
    m1_val &= ~(1 << pin_bit);   // M1 = 0
    m2_val |= (1 << pin_bit);    // M2 = 1
    
    // レジスタに書き戻し
    ret |= write_register(encoder, reg_m1, m1_val);
    ret |= write_register(encoder, reg_m2, m2_val);
    
    // PWMチャンネル有効化
    uint8_t piocon_reg;
    uint8_t piocon_bit;
    
    switch (pwm_channel) {
        case 0:  piocon_reg = REG_PIOCON0; piocon_bit = 2; break;
        case 1:  piocon_reg = REG_PIOCON0; piocon_bit = 1; break;
        case 2:  piocon_reg = REG_PIOCON0; piocon_bit = 0; break;
        case 3:  piocon_reg = REG_PIOCON0; piocon_bit = 3; break;
        case 4:  piocon_reg = REG_PIOCON1; piocon_bit = 0; break;
        case 5:  piocon_reg = REG_PIOCON1; piocon_bit = 1; break;
        default:
            ESP_LOGE(TAG, "無効なPWMチャンネル: %d", pwm_channel);
            return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t piocon_val;
    ret |= read_register(encoder, piocon_reg, &piocon_val);
    piocon_val |= (1 << piocon_bit);
    ret |= write_register(encoder, piocon_reg, piocon_val);
    
    // PWM極性設定（共通カソードLED用の反転）
    if (invert) {
        uint8_t pnp_reg;
        ret |= read_register(encoder, REG_PNP, &pnp_reg);
        pnp_reg |= (1 << pwm_channel);
        ret |= write_register(encoder, REG_PNP, pnp_reg);
    }
    
    return ret;
}

/**
 * @brief ロータリーエンコーダーハードウェア設定
 */
static esp_err_t setup_rotary_encoder_hardware(pimoroni_encoder_t *encoder)
{
    esp_err_t ret = ESP_OK;

    // エンコーダーピンの設定値作成
    uint8_t encoder_config = ENC_TERM_A | (ENC_TERM_B << 4);

    // エンコーダー設定レジスタに書き込み
    ret |= write_register(encoder, REG_ENC_1_CFG, encoder_config);

    // エンコーダー有効化
    uint8_t enc_enable = 0x01; // チャンネル1有効
    ret |= write_register(encoder, REG_ENC_EN, enc_enable);

    // エンコーダーカウントをゼロリセット
    ret |= write_register(encoder, REG_ENC_1_COUNT, 0x00);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "エンコーダー初期化完了 (A=%d, B=%d)", ENC_TERM_A, ENC_TERM_B);
    }

    return ret;
}

/**
 * @brief PWMシステム設定（シンプル版）
 */
static esp_err_t setup_pwm_system(pimoroni_encoder_t *encoder)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "PWMシステム初期化開始");

    // PWM周期計算（明度に応じて）
    uint16_t pwm_period = (uint16_t)(255.0f / encoder->brightness);

    // PWM周期設定
    ret |= write_register(encoder, REG_PWMPL, (uint8_t)(pwm_period & 0xFF));
    ret |= write_register(encoder, REG_PWMPH, (uint8_t)(pwm_period >> 8));

    // PWM制御設定
    ret |= write_register(encoder, REG_PWMCON1, 0x01); // 分周比=2

    // RGB LEDピンをPWMモードに設定
    ret |= set_pin_mode_pwm(encoder, LED_R_PHYSICAL_PIN, LED_R_PWM_CHANNEL, true);
    ret |= set_pin_mode_pwm(encoder, LED_G_PHYSICAL_PIN, LED_G_PWM_CHANNEL, true);
    ret |= set_pin_mode_pwm(encoder, LED_B_PHYSICAL_PIN, LED_B_PWM_CHANNEL, true);

    // PWM実行開始
    uint8_t pwmcon0 = 0x80; // PWMRUNビット
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);

    // PWM設定をロード
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40; // LOADビット設定
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "PWMシステム初期化完了");
    }

    return ret;
}

/**
 * @brief 割り込み出力有効化（スキップ版）
 */
static esp_err_t enable_interrupt_output(pimoroni_encoder_t *encoder)
{
    // 割り込みピンを使用しないのでスキップ
    ESP_LOGD(TAG, "割り込みピン未使用のためスキップ");
    return ESP_OK;
}

// ===== 公開API実装 =====

esp_err_t pimoroni_encoder_init(pimoroni_encoder_t *encoder, const pimoroni_encoder_config_t *config)
{
    if (!encoder || !config)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Pimoroni Encoder初期化開始");

    // 構造体初期化
    memset(encoder, 0, sizeof(pimoroni_encoder_t));
    encoder->i2c_port = config->i2c_port;
    encoder->i2c_address = config->i2c_address;
    encoder->interrupt_pin = config->interrupt_pin;
    encoder->direction = config->direction;
    encoder->brightness = config->brightness;
    encoder->encoder_offset = 0;
    encoder->encoder_last = 0;
    encoder->encoder_raw_last = 0;          // 追加：生の値の初期化
    encoder->encoder_accumulator = 0.0f;     // 追加：累積カウンタの初期化
    encoder->encoder_accumulator_time = 0;   // 追加：累積時刻の初期化

    // タスク管理用メンバの初期化
    encoder->poll_task_handle = NULL;
    encoder->current_value = 0;
    encoder->last_direction = 0;
    encoder->last_raw_diff = 0;
    encoder->last_change_time = 0;
    encoder->initialized = false;

    // 明度値の範囲チェック
    if (encoder->brightness < 0.01f)
        encoder->brightness = 0.01f;
    if (encoder->brightness > 1.0f)
        encoder->brightness = 1.0f;

    esp_err_t ret = ESP_OK;

    // チップID確認（スキップ指定がない場合）
    if (!config->skip_chip_id_check)
    {
        uint16_t chip_id = pimoroni_encoder_get_chip_id(encoder);
        if (chip_id != PIMORONI_ENCODER_CHIP_ID)
        {
            ESP_LOGE(TAG, "チップID不正: 0x%04X (期待値: 0x%04X)", chip_id, PIMORONI_ENCODER_CHIP_ID);
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "チップID確認OK: 0x%04X", chip_id);
    }

    // ハードウェア初期化
    ret |= setup_rotary_encoder_hardware(encoder);
    ret |= setup_pwm_system(encoder);
    ret |= enable_interrupt_output(encoder);

    if (ret == ESP_OK)
    {
        // 初期化フラグを立てる（ポーリングタスク開始前に必須）
        encoder->initialized = true;

        // 内部ポーリングタスクを起動
        BaseType_t task_ret = xTaskCreate(
            encoder_poll_task,
            "encoder_poll",
            4096,
            encoder,
            tskIDLE_PRIORITY + 4,  // 優先度4（main.cppのencoder_taskと同等）
            &encoder->poll_task_handle
        );

        if (task_ret != pdPASS)
        {
            ESP_LOGE(TAG, "ポーリングタスク作成失敗");
            encoder->initialized = false;
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Pimoroni Encoder初期化完了 (I2C=0x%02X, ポーリングタスク起動)", encoder->i2c_address);
    }
    else
    {
        ESP_LOGE(TAG, "Pimoroni Encoder初期化失敗");
    }

    return ret;
}

esp_err_t pimoroni_encoder_deinit(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // ポーリングタスクを停止
    if (encoder->poll_task_handle != NULL)
    {
        encoder->initialized = false;  // タスクループを終了させる
        vTaskDelay(pdMS_TO_TICKS(50)); // タスク終了を待つ
        encoder->poll_task_handle = NULL;
        ESP_LOGI(TAG, "ポーリングタスク停止");
    }

    // エンコーダー無効化
    write_register(encoder, REG_ENC_EN, 0x00);

    // PWM停止
    write_register(encoder, REG_PWMCON0, 0x00);

    // LED消灯
    pimoroni_encoder_set_led(encoder, 0, 0, 0);

    ESP_LOGI(TAG, "Pimoroni Encoder破棄完了");
    return ESP_OK;
}

/**
 * @brief エンコーダー読み取り（修正版：シンプルかつ動作保証）
 */
int16_t pimoroni_encoder_read(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        ESP_LOGE(TAG, "無効なエンコーダーポインタ");
        return 0;
    }

    // レジスタから値を読み取り
    uint8_t raw_count;
    esp_err_t ret = read_register(encoder, REG_ENC_1_COUNT, &raw_count);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "エンコーダー読み取りエラー");
        return encoder->encoder_offset + encoder->encoder_last;
    }

    // 差分計算（前回からの変化量）
    int8_t raw_diff = (int8_t)(raw_count - encoder->encoder_raw_last);
    encoder->encoder_raw_last = raw_count;

    // 生のdiffを常に保存（ポーリングタスクでチャタリング検出に使用）
    // 重要: 0の場合も保存して、古い値が残らないようにする
    encoder->last_raw_diff = raw_diff;

    // 差分が0の場合は何もせずに戻る
    if (raw_diff == 0) {
        int16_t total_count = encoder->encoder_offset + encoder->encoder_last;
        if (encoder->direction == PIMORONI_ENCODER_CCW) {
            total_count = -total_count;
        }
        return total_count;
    }

    // 差分がある場合のみログ出力
    ESP_LOGI(TAG, "エンコーダー変化検出: raw=0x%02X, diff=%d", raw_count, raw_diff);

    // 現在時刻を取得（タイムアウト判定用）
    uint32_t current_time = esp_timer_get_time() / 1000;  // ミリ秒単位

    // 累積フィルタ: ±1の小刻みな変化を吸収
    encoder->encoder_accumulator += (float)raw_diff;

    // 累積値が変化した時刻を記録（タイムアウト判定用）
    if (encoder->encoder_accumulator != 0.0f) {
        if (encoder->encoder_accumulator_time == 0) {
            encoder->encoder_accumulator_time = current_time;  // 初回累積開始
        }
    }

    bool updated = false;
    int accumulated_change = 0;

    // 累積値が±1以上になったら encoder_last を更新（閾値を±2→±1に変更）
    while (encoder->encoder_accumulator >= 1.0f)
    {
        encoder->encoder_last += 1;
        encoder->encoder_accumulator -= 1.0f;
        accumulated_change += 1;
        updated = true;
    }
    while (encoder->encoder_accumulator <= -1.0f)
    {
        encoder->encoder_last -= 1;
        encoder->encoder_accumulator += 1.0f;
        accumulated_change -= 1;
        updated = true;
    }

    // タイムアウトチェック: 500ms以上累積されたままの場合、強制的に反映
    #define ACCUMULATOR_TIMEOUT_MS 500
    if (encoder->encoder_accumulator != 0.0f && encoder->encoder_accumulator_time != 0) {
        uint32_t elapsed = current_time - encoder->encoder_accumulator_time;
        if (elapsed >= ACCUMULATOR_TIMEOUT_MS) {
            // タイムアウト: 残りの累積値を四捨五入して反映
            int timeout_change = (int)roundf(encoder->encoder_accumulator);
            if (timeout_change != 0) {
                encoder->encoder_last += timeout_change;
                accumulated_change += timeout_change;
                ESP_LOGI(TAG, "タイムアウト: 累積残り%.2f → %d を強制反映 (%ldms経過)",
                        (double)encoder->encoder_accumulator, timeout_change, elapsed);
                encoder->encoder_accumulator = 0.0f;
                encoder->encoder_accumulator_time = 0;
                updated = true;
            }
        }
    }

    // 累積値がゼロになったら時刻をリセット
    if (encoder->encoder_accumulator == 0.0f) {
        encoder->encoder_accumulator_time = 0;
    }

    if (updated) {
        ESP_LOGI(TAG, "累積フィルタ: 累積diff=%d, last=%d, 残り=%.2f",
                accumulated_change, encoder->encoder_last, (double)encoder->encoder_accumulator);
    } else {
        ESP_LOGD(TAG, "累積中: acc=%.2f (±1未満のため保留)", (double)encoder->encoder_accumulator);
    }

    // オーバーフロー検出
    if (encoder->encoder_last > 127)
    {
        encoder->encoder_offset += 256;
        encoder->encoder_last -= 256;
        ESP_LOGW(TAG, "正方向OF: offset=%d", encoder->encoder_offset);
    }
    else if (encoder->encoder_last < -128)
    {
        encoder->encoder_offset -= 256;
        encoder->encoder_last += 256;
        ESP_LOGW(TAG, "負方向UF: offset=%d", encoder->encoder_offset);
    }

    // 最終カウント値計算
    int16_t total_count = encoder->encoder_offset + encoder->encoder_last;

    // 回転方向を考慮
    if (encoder->direction == PIMORONI_ENCODER_CCW)
    {
        total_count = -total_count;
    }

    // 値が更新された場合のみ最終値を表示
    if (updated) {
        ESP_LOGI(TAG, "エンコーダー値更新: %d (offset=%d, last=%d, acc=%.2f)",
                total_count, encoder->encoder_offset, encoder->encoder_last,
                (double)encoder->encoder_accumulator);
    }

    return total_count;
}

/**
 * @brief 内部ポーリングタスク
 *
 * このタスクはドライバ初期化時に自動起動され、以下の処理を行います：
 * - エンコーダー値の定期的なポーリング
 * - 生のraw_diffレベルでの逆方向±1チャタリング除去
 * - 方向ベースのデバウンス処理
 * - アキュムレータフィルタとタイムアウト処理
 */
static void encoder_poll_task(void *arg)
{
    pimoroni_encoder_t *encoder = (pimoroni_encoder_t *)arg;

    ESP_LOGI(TAG, "エンコーダーポーリングタスク開始");

    int8_t last_raw_direction = 0;  // 生のraw_diffから判定した最後の確定方向
    int8_t opposite_count = 0;       // 逆方向カウント（累積±2で方向転換確定）
    int8_t opposite_sign = 0;        // 逆方向の符号

    while (encoder->initialized)
    {
        // エンコーダー値を読み取り（内部フィルタ適用済み）
        // この中でlast_raw_diffが更新される
        int16_t raw_value = pimoroni_encoder_read(encoder);
        int8_t raw_diff = encoder->last_raw_diff;

        // 生のdiffがある場合のみ処理
        if (raw_diff != 0)
        {
            // 生のraw_diffから方向を判定
            int8_t raw_direction = (raw_diff > 0) ? 1 : -1;
            ESP_LOGD(TAG, "ポーリング: raw_diff=%d, raw_dir=%d, last_raw_dir=%d",
                    raw_diff, raw_direction, last_raw_direction);

            // チャタリング検出: 逆方向の累積カウントで判定
            if (last_raw_direction != 0 &&
                raw_direction != last_raw_direction &&
                abs(raw_diff) == 1)
            {
                // 逆方向±1を検出 → カウントアップ
                if (opposite_sign == 0 || opposite_sign == raw_direction)
                {
                    // 同じ逆方向に累積
                    opposite_sign = raw_direction;
                    opposite_count++;

                    if (opposite_count >= 2)
                    {
                        // 累積±2以上 → 本物の方向転換と判定
                        ESP_LOGI(TAG, "方向転換確定: %s → %s (累積%d回)",
                                last_raw_direction > 0 ? "CW" : "CCW",
                                raw_direction > 0 ? "CW" : "CCW",
                                opposite_count);
                        last_raw_direction = raw_direction;
                        opposite_count = 0;
                        opposite_sign = 0;
                    }
                    else
                    {
                        // まだ累積中 → 保留
                        ESP_LOGD(TAG, "逆方向累積中: count=%d/2", opposite_count);
                        vTaskDelay(pdMS_TO_TICKS(ENCODER_POLL_INTERVAL_MS));
                        continue;
                    }
                }
                else
                {
                    // 違う逆方向 → カウントリセット
                    ESP_LOGD(TAG, "逆方向切替: カウントリセット");
                    opposite_sign = raw_direction;
                    opposite_count = 1;
                    vTaskDelay(pdMS_TO_TICKS(ENCODER_POLL_INTERVAL_MS));
                    continue;
                }
            }
            else
            {
                // 同方向 or 初回 → カウントをクリアして方向を更新
                if (opposite_count > 0)
                {
                    ESP_LOGD(TAG, "同方向復帰: 逆方向カウントクリア(%d)", opposite_count);
                }
                opposite_count = 0;
                opposite_sign = 0;
                last_raw_direction = raw_direction;
            }

            // フィルタ済みの値の変化をチェック
            int16_t delta = raw_value - encoder->current_value;

            if (delta != 0)
            {
                // 現在の方向を判定（フィルタ済みdeltaから）
                int8_t current_direction = (delta > 0) ? 1 : -1;
                uint32_t current_time = esp_timer_get_time() / 1000; // μs → ms

                // デバウンス処理
                bool should_update = false;

                if (encoder->last_direction == 0)
                {
                    // 初回の変化は常に受け入れ
                    should_update = true;
                    ESP_LOGI(TAG, "初回変化: delta=%d, raw_diff=%d", delta, raw_diff);
                }
                else if (current_direction == encoder->last_direction)
                {
                    // 同方向への変化
                    uint32_t elapsed = current_time - encoder->last_change_time;
                    if (elapsed >= ENCODER_DEBOUNCE_SAME_DIR_MS)
                    {
                        should_update = true;
                        ESP_LOGI(TAG, "同方向変化OK: delta=%d, raw_diff=%d, elapsed=%lums", delta, raw_diff, elapsed);
                    }
                    else
                    {
                        ESP_LOGD(TAG, "同方向デバウンス中: elapsed=%lums < %dms", elapsed, ENCODER_DEBOUNCE_SAME_DIR_MS);
                    }
                }
                else
                {
                    // 逆方向への変化（ここまで来るのは±2以上の変化のみ）
                    uint32_t elapsed = current_time - encoder->last_change_time;

                    if (elapsed >= ENCODER_DEBOUNCE_OPPOSITE_DIR_MS)
                    {
                        should_update = true;
                        ESP_LOGI(TAG, "逆方向変化OK: delta=%d, raw_diff=%d, elapsed=%lums", delta, raw_diff, elapsed);
                    }
                    else
                    {
                        ESP_LOGD(TAG, "逆方向デバウンス中: elapsed=%lums < %dms", elapsed, ENCODER_DEBOUNCE_OPPOSITE_DIR_MS);
                    }
                }

                // 値を更新
                if (should_update)
                {
                    encoder->current_value = raw_value;
                    encoder->last_direction = current_direction;
                    encoder->last_change_time = current_time;

                    ESP_LOGI(TAG, "ドライバ値更新: %d → %d (delta=%d, raw_diff=%d, dir=%s)",
                            encoder->current_value - delta, encoder->current_value, delta, raw_diff,
                            current_direction > 0 ? "CW" : "CCW");
                }
            }
        }

        // ポーリング間隔で待機
        vTaskDelay(pdMS_TO_TICKS(ENCODER_POLL_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "エンコーダーポーリングタスク終了");
    vTaskDelete(NULL);
}

/**
 * @brief アプリケーション向けAPI: 安定したエンコーダー値を取得
 *
 * @param encoder エンコーダーデバイス構造体
 * @return フィルタリング・デバウンス処理済みの安定した値
 */
int16_t pimoroni_encoder_get_value(pimoroni_encoder_t *encoder)
{
    if (!encoder || !encoder->initialized)
    {
        return 0;
    }

    return encoder->current_value;
}

esp_err_t pimoroni_encoder_clear(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return ESP_ERR_INVALID_ARG;
    }

    encoder->encoder_offset = 0;
    encoder->encoder_last = 0;
    encoder->encoder_raw_last = 0;          // 追加
    encoder->encoder_accumulator = 0.0f;     // 追加
    encoder->encoder_accumulator_time = 0;   // 追加

    // タスク管理用メンバもクリア
    encoder->current_value = 0;
    encoder->last_direction = 0;
    encoder->last_raw_diff = 0;
    encoder->last_change_time = 0;

    esp_err_t ret = write_register(encoder, REG_ENC_1_COUNT, 0x00);
    if (ret == ESP_OK)
    {
        ESP_LOGD(TAG, "エンコーダーカウントクリア完了（累積含む）");
    }

    return ret;
}

esp_err_t pimoroni_encoder_set_direction(pimoroni_encoder_t *encoder, pimoroni_encoder_direction_t direction)
{
    if (!encoder)
    {
        return ESP_ERR_INVALID_ARG;
    }

    encoder->direction = direction;
    ESP_LOGD(TAG, "エンコーダー方向設定: %s",
             (direction == PIMORONI_ENCODER_CW) ? "時計回り" : "反時計回り");

    return ESP_OK;
}

pimoroni_encoder_direction_t pimoroni_encoder_get_direction(const pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return PIMORONI_ENCODER_CW;
    }

    return encoder->direction;
}

/**
 * @brief RGB LEDの色を設定（公式準拠・最適化版）
 */
esp_err_t pimoroni_encoder_set_led(pimoroni_encoder_t *encoder, uint8_t r, uint8_t g, uint8_t b)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    // RGB値をPWM値にスケーリング（明度考慮）
    uint16_t r_pwm = (uint16_t)(r * encoder->brightness);
    uint16_t g_pwm = (uint16_t)(g * encoder->brightness);
    uint16_t b_pwm = (uint16_t)(b * encoder->brightness);
    
    ESP_LOGD(TAG, "LED設定: RGB(%d,%d,%d) → PWM(%d,%d,%d)", r, g, b, r_pwm, g_pwm, b_pwm);
    
    // 公式準拠: PWM値を設定（ロード保留）
    // 赤LED: PWMチャンネル5
    ret |= write_register(encoder, REG_PWM5L, (uint8_t)(r_pwm & 0xFF));
    ret |= write_register(encoder, REG_PWM5H, (uint8_t)(r_pwm >> 8));
    
    // 緑LED: PWMチャンネル1
    ret |= write_register(encoder, REG_PWM1L, (uint8_t)(g_pwm & 0xFF));
    ret |= write_register(encoder, REG_PWM1H, (uint8_t)(g_pwm >> 8));
    
    // 青LED: PWMチャンネル2
    ret |= write_register(encoder, REG_PWM2L, (uint8_t)(b_pwm & 0xFF));
    ret |= write_register(encoder, REG_PWM2H, (uint8_t)(b_pwm >> 8));
    
    // 公式準拠: 全てのPWMを一度にロード（ちらつき防止）
    uint8_t pwmcon0;
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40;  // LOADビット設定で全PWM同期更新
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);
    
    return ret;
}

esp_err_t pimoroni_encoder_set_brightness(pimoroni_encoder_t *encoder, float brightness)
{
    if (!encoder)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 明度値の範囲制限
    if (brightness < 0.01f)
        brightness = 0.01f;
    if (brightness > 1.0f)
        brightness = 1.0f;

    encoder->brightness = brightness;

    // PWM周期を再計算・設定
    uint16_t pwm_period = (uint16_t)(255.0f / brightness);
    esp_err_t ret = ESP_OK;

    ret |= write_register(encoder, REG_PWMPL, (uint8_t)(pwm_period & 0xFF));
    ret |= write_register(encoder, REG_PWMPH, (uint8_t)(pwm_period >> 8));

    // PWM設定をロード
    uint8_t pwmcon0;
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40; // LOADビット設定
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);

    if (ret == ESP_OK)
    {
        ESP_LOGD(TAG, "明度設定完了: %.2f", (double)brightness);
    }

    return ret;
}

/**
 * @brief 割り込みフラグ確認（割り込みピン無し版）
 */
bool pimoroni_encoder_available(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return false;
    }

    // 割り込みピンを使用しないので常にtrue
    // エンコーダーは常にポーリングで読み取り可能
    return true;
}

/**
 * @brief 割り込みフラグクリア（割り込みピン無し版）
 */
esp_err_t pimoroni_encoder_clear_interrupt(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 割り込みピンを使用しないので何もしない
    return ESP_OK;
}

uint16_t pimoroni_encoder_get_chip_id(pimoroni_encoder_t *encoder)
{
    if (!encoder)
    {
        return 0;
    }

    uint16_t chip_id;
    esp_err_t ret = read_register_16bit(encoder, REG_CHIP_ID_L, REG_CHIP_ID_H, &chip_id);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "チップID読み取りエラー");
        return 0;
    }

    return chip_id;
}

pimoroni_encoder_config_t pimoroni_encoder_get_default_config(i2c_port_t i2c_port)
{
    pimoroni_encoder_config_t config = {
        .i2c_port = i2c_port,
        .i2c_address = PIMORONI_ENCODER_I2C_ADDR,
        .interrupt_pin = GPIO_NUM_NC, // 割り込みピン未使用
        .direction = PIMORONI_ENCODER_CW,
        .brightness = 1.0f,
        .skip_chip_id_check = false};

    return config;
}