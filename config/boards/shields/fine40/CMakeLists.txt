# NINO shield sources.
#
# ZMK's Zephyr fork patches boards/CMakeLists.txt to add_subdirectory() a
# shield's CMakeLists.txt, including out-of-tree shields like this one, so these
# sources are compiled without needing a separate Zephyr module.

if(CONFIG_ZMK_DISPLAY AND CONFIG_NINO_DISPLAY)
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
  zephyr_library_sources(custom_status_screen.c)
  zephyr_library_sources(widgets/nino.c)
endif()

if(CONFIG_NINO_NOTES)
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
  zephyr_library_sources(notes/nino_store.c)
  zephyr_library_sources(notes/nino_link.c)
endif()
