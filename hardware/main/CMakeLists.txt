idf_component_register(SRCS "src/hardware.cpp" "src/network.cpp"
                           "src/mlx90640_i2c.cpp"
                           "../../protocol/src/protocol/lidar_imu_odometry.cpp"
                           "../../protocol/src/protocol/motion_control.cpp"
                           "third_party/melexis_mlx90640/functions/MLX90640_API.c"
                    INCLUDE_DIRS "include" "../../protocol/include"
                                 "third_party/melexis_mlx90640/headers"
                    PRIV_REQUIRES esp_driver_i2c esp_driver_uart esp_driver_usb_serial_jtag esp_event esp_hw_support esp_wifi nvs_flash esp_driver_gpio esp_driver_ledc esp_timer)
