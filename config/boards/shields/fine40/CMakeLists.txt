# fine!40 shield sources.
#
# ZMK's Zephyr fork patches boards/CMakeLists.txt to add_subdirectory() a
# shield's CMakeLists.txt, including out-of-tree shields like this one, so these
# sources are compiled without needing a separate Zephyr module.

if(CONFIG_ZMK_DISPLAY AND CONFIG_FINE40_RIPPLE)
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
  zephyr_library_sources(custom_status_screen.c)
  zephyr_library_sources(widgets/ripple.c)
endif()
