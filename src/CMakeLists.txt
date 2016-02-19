rock_library(uwv_filters
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(uwv_filters_bin Main.cpp
    DEPS uwv_filters)

