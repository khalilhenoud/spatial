cmake_minimum_required(VERSION 3.22)

project(spatial_package)

if (NOT HAS_STANDALONE_PREDECESSOR)
  # used to omit test projects from dependencies.
  set(HAS_STANDALONE_PREDECESSOR True)

  # DOWNLOAD ALL THE SUBMODULES
  find_package(Git QUIET)
  if (GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if (GIT_SUBMODULES)
      message(STATUS "Submodule update")
      execute_process(COMMAND $(GIT_EXECUTABLE) submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
      if (NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}")
      endif()
    endif()
  endif()

  # CHECK ALL THE SUBMODULES
  if (NOT EXISTS "${PROJECT_SOURCE_DIR}/external/math/CMakeLists.txt")
    message(FATAL_ERROR "The math submdule was not downloaded!")
  endif()

  # CHECK ALL THE SUBMODULES
  if (NOT EXISTS "${PROJECT_SOURCE_DIR}/external/library/CMakeLists.txt")
    message(FATAL_ERROR "The library submdule was not downloaded!")
  endif()

  #add the subdirectory
  add_subdirectory(external/math math)

  #add the subdirectory
  add_subdirectory(external/library library)
endif()

add_subdirectory(spatial)