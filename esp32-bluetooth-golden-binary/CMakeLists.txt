cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Disable the "Trim" build.
idf_build_set_property(MINIMAL_BUILD OFF)

#add_compile_options(-g -gdwarf-2)
#add_link_options(-g)

project(esp32-bluetooth-golden-binary)

idf_build_get_property(all_compile_options COMPILE_OPTIONS)

# Remove any -gdwarf-* that might have been injected
list(FILTER all_compile_options EXCLUDE REGEX "^-gdwarf-[0-9]+$")

# Apply filtered options back
idf_build_set_property(COMPILE_OPTIONS "${all_compile_options}" APPEND)

# Now append our preferred setting last so it wins
idf_build_set_property(COMPILE_OPTIONS "-gdwarf-2" APPEND)



