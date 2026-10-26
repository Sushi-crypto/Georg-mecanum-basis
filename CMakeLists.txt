cmake_minimum_required(VERSION 3.16)
project(MecanumRobotTests CXX C)

set(CMAKE_CXX_STANDARD 14)

# 1. Suche nach dem installierten CppUTest-Paket im Linux-Container
find_package(PkgConfig REQUIRED)

# 2. CppUTest-Bibliothek aufspüren
pkg_check_modules(CPPUTEST REQUIRED cpputest)

include_directories(${CPPUTEST_INCLUDE_DIRS})
include_directories(include)
# 2. Erstelle das Test-Programm aus unserer C++-Datei
add_executable(smoke_test tests/SmokeTest.cpp)

# 3. Verbinde das Test-Programm mit der CppUTest-Bibliothek
target_link_libraries(smoke_test ${CPPUTEST_LIBRARIES})