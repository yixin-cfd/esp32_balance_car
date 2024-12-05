file(GLOB_RECURSE SRCS_LIST "*.c" "./eMPL/*.c")

idf_component_register(SRCS ${SRCS_LIST}
                       INCLUDE_DIRS "." ".."
                       REQUIRES driver freertos)


# idf_component_register(SRCS "mpu6050.c" 
#                        INCLUDE_DIRS "." "eMPL"
#                        REQUIRES driver freertos)
