set(src_dirs
    i2c
    mpu9250
)

set(include_dirs
    i2c
    mpu9250
    imu_sensor
)

set(requires
    driver
    esp_timer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
