idf_component_register(SRCS "odometry.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES object_utils
                             nvs_flash)
