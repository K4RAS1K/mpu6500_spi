set(srcs "main.c"
         "mpu6500_lib.c"
         "spi_driver.c"
         "MadgwickAHRS/MadgwickAHRS.c"
         #"MahonyAHRS/MahonyAHRS.c"
         "MovingAverage/MovingAverage.c"
         "HighPassFilter/HighPassFilter.c"
         "Q_to_a/quartirions_to_angels.c")
         
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash
                       REQUIRES driver)
