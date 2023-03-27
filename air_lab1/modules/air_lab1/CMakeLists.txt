
march_generate_module(lrs_march_air_lab1 ${CMAKE_CURRENT_SOURCE_DIR}/../air_lab1.mal ${CMAKE_CURRENT_BINARY_DIR}/ PROTOTYPES_DIRECTORY ${LRS_MARCH_PROTOTYPES_DIR} )

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ops.h
                    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ops.h
                    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/ops.h ${CMAKE_CURRENT_BINARY_DIR}/ops.h)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/defs.h
                    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/defs.h
                    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/defs.h ${CMAKE_CURRENT_BINARY_DIR}/defs.h)

add_dependencies(march_air_lab1_modules lrs_march_air_lab1)
