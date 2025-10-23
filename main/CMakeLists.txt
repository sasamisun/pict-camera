idf_component_register(
    SRCS
        "main.cpp"
       
        #"PixelArtProcessor.cpp"
        "camera_utils.cpp"
        "ssd1306_display.cpp"
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
    PRIV_REQUIRES
        "M5GFX"                       # M5Stack用グラフィックライブラリ
        
)