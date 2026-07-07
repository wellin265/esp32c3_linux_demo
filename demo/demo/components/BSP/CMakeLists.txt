set(src_dirs
            LED
            )

set(include_dirs
            LED
            )

set(requires
            esp_driver_gpio
            )

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-format)