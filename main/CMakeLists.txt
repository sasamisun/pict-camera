idf_component_register(
    SRCS
        "main.cpp"
        "PimoroniEncoder.cpp"
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
        "log"
    PRIV_REQUIRES
        "M5GFX"                       # M5Stack用グラフィックライブラリ
        
)