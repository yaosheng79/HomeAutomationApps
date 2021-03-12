#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"
#include "camera_pins.h"

// macros for motion detection
#define FRAME_SIZE FRAMESIZE_QQVGA
#define WIDTH 160
#define HEIGHT 120
#define BLOCK_SIZE 8
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
//#define DEBUG 1

uint16_t prev_frame[H][W] = { 0 };
uint16_t current_frame[H][W] = { 0 };

bool config_camera(pixformat_t, framesize_t);
bool capture_still();
void capture_jpeg();
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);

bool config_camera(pixformat_t format, framesize_t frameSize) {
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor->set_pixformat(sensor, format) !=0) {
      Serial.println("set_format Failed");
      return false;
    }
    if (sensor->set_framesize(sensor, frameSize) !=0) {
      Serial.println("set_framesize Failed");
      return false;
    }
}

/**
 *
 */
void setup() {
    Serial.begin(115200);

    camera_config_t camera_config;
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.pin_d0 = Y2_GPIO_NUM;
    camera_config.pin_d1 = Y3_GPIO_NUM;
    camera_config.pin_d2 = Y4_GPIO_NUM;
    camera_config.pin_d3 = Y5_GPIO_NUM;
    camera_config.pin_d4 = Y6_GPIO_NUM;
    camera_config.pin_d5 = Y7_GPIO_NUM;
    camera_config.pin_d6 = Y8_GPIO_NUM;
    camera_config.pin_d7 = Y9_GPIO_NUM;
    camera_config.pin_xclk = XCLK_GPIO_NUM;
    camera_config.pin_pclk = PCLK_GPIO_NUM;
    camera_config.pin_vsync = VSYNC_GPIO_NUM;
    camera_config.pin_href = HREF_GPIO_NUM;
    camera_config.pin_sscb_sda = SIOD_GPIO_NUM;
    camera_config.pin_sscb_scl = SIOC_GPIO_NUM;
    camera_config.pin_pwdn = PWDN_GPIO_NUM;
    camera_config.pin_reset = RESET_GPIO_NUM;
    camera_config.xclk_freq_hz = 20000000;
    camera_config.pixel_format = PIXFORMAT_GRAYSCALE;
    camera_config.frame_size = FRAME_SIZE;
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 1;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    if (!config_camera(PIXFORMAT_GRAYSCALE, FRAME_SIZE)) {
      Serial.println("ERR INIT GRAYSCALE");
      return;
    }
}

/**
 *
 */
void loop() {
    if (!capture_still()) {
        Serial.println("Failed capture");
        delay(3000);

        return;
    }

    if (motion_detect()) {
      Serial.println();
      Serial.println("Motion detected");
      capture_jpeg();
    }

    update_frame();
    Serial.print(".");
}

void capture_jpeg() {
  int64_t fr_start = esp_timer_get_time();
  if (!config_camera(PIXFORMAT_JPEG, FRAMESIZE_VGA)) {
    Serial.println("ERR INIT JPEG");
    return;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
      Serial.println("Capture jpeg failed");
      return;
  }
  if(fb->format != PIXFORMAT_JPEG){
    Serial.println("wrong format");
    return;
  }
  size_t fb_len = fb->len;
  if (!config_camera(PIXFORMAT_GRAYSCALE, FRAME_SIZE)) {
    Serial.println("ERR INIT GRAYSCALE");
    return;
  }
  int64_t fr_end = esp_timer_get_time();
  Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
}
/**
 * Capture image and do down-sampling
 */
bool capture_still() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }
    // set all 0s in current frame
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = fb->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    print_frame(current_frame);
#endif

    return true;
}


/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */
bool motion_detect() {
    uint16_t changes = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;

            if (delta >= BLOCK_DIFF_THRESHOLD) {
#if DEBUG
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif

                changes += 1;
            }
        }
    }
#if DEBUG
    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);
#endif
    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}


/**
 * Copy current frame to previous
 */
void update_frame() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            prev_frame[y][x] = current_frame[y][x];
        }
    }
}

/**
 * For serial debugging
 * @param frame
 */
void print_frame(uint16_t frame[H][W]) {
    Serial.println("Current frame:");
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
    Serial.println("---------------");
}
