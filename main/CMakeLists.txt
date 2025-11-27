idf_component_register(
    SRCS
        "main.cpp"
        "pimoroni_encoder.cpp"
        #"PixelArtProcessor.cpp"
        "camera_utils.cpp"
        "ssd1306_display.cpp"
        "terminal.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        "esp32-camera"
        "driver"
        "fatfs"
        "spi_flash"
        "nvs_flash"
        "freertos"
        "esp_system"
        "esp_common"
        "heap"
        "log"
        "esp_tinyusb"
    PRIV_REQUIRES
        "M5GFX"                       # M5Stack用グラフィックライブラリ
        
)