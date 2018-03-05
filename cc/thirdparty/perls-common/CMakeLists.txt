file(GLOB implem_files *.c)
file(GLOB header_files *.h)

set(LIB_NAME perls-common)

add_library (${LIB_NAME} SHARED ${implem_files})

# non-pkg-config libraries go here
set(REQUIRED_LIBS -lgsl -latlas -lcblas -lm)
target_link_libraries(${LIB_NAME} ${REQUIRED_LIBS})

# pkg-config packages go here
set(REQUIRED_PACAKGES bot2-core bot2-param-client lcm glib-2.0 lcmtypes_senlcm)
pods_use_pkg_config_packages (${LIB_NAME} ${REQUIRED_PACAKGES})

pods_install_libraries(${LIB_NAME})
pods_install_headers(${header_files} DESTINATION ${LIB_NAME})
pods_install_pkg_config_file(${LIB_NAME}
    LIBS -l${LIB_NAME} ${REQUIRED_LIBS}
    REQUIRES ${REQUIRED_PACAKGES}
    VERSION 0.0.1)
