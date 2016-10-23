# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

IF (ENABLE_ManipWaiterYarp)

  option(ENABLE_launchManipWaiterYarp "Choose if you want to compile launchManipWaiterYarp" TRUE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
  include(YarpPlugin)

  # Exporting dependencies for TEOConfig.cmake quite manually for now... 
  #set(MANIP_WAITER_LIBRARIES ${MANIP_WAITER_LIBRARIES} TeoYarp CACHE INTERNAL "appended libraries")

  yarp_begin_plugin_library(ManipWaiterYarp)
    add_subdirectory(KdlSolver)
  yarp_end_plugin_library(ManipWaiterYarp)

  install(TARGETS ManipWaiterYarp DESTINATION lib)

  IF (ENABLE_launchManipWaiterYarp)

    yarp_add_plugin_library_executable(launchManipWaiterYarp ${MANIP_WAITER_LIBRARIES})
    install(TARGETS launchManipWaiterYarp DESTINATION bin)

  ENDIF (ENABLE_launchManipWaiterYarp)

ENDIF (ENABLE_ManipWaiterYarp)


