file(GLOB hpp_files *.hpp)

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est/mav-est-realsense)

# make a state estimator library 
add_library(mav-est-realsense SHARED 
                rbis_realsense_update.cpp )

set(REQUIRED_LIBS mav-state-est 
    bot2-core
	bot2-param-client
	bot2-frames
	eigen-utils
    path-util
    pronto_vis
    )
#	laser-util
#	laser-sim3d


pods_use_pkg_config_packages(mav-est-realsense ${REQUIRED_LIBS})
pods_install_libraries(mav-est-realsense)
pods_install_pkg_config_file(mav-est-realsense
    CFLAGS
    LIBS -lmav-est-realsense
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)
