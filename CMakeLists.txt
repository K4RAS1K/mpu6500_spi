set(srcs "main.c"
         "mpu6500_lib.c"
         "spi_driver.c"
<<<<<<< HEAD
         "MadgwickAHRS/MadgwickAHRS.c"
         #"MahonyAHRS/MahonyAHRS.c"
         "MovingAverage/MovingAverage.c"
         "HighPassFilter/HighPassFilter.c")
         
=======
         "MadgwickAHRS/MadgwickAHRS.c")
>>>>>>> d3f1c12d75070e5db16228437c96b7d27941ea7c
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash
                       REQUIRES driver)
