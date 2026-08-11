cmake_minimum_required(VERSION 3.16)
project(RTPCP CXX)

# ===== 全局编译器选项与 C++ 标准 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3 /O2 /EHsc /utf-8)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ===== 依赖库查找 =====
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找 LASzip (通过 FindLASZIP / vcpkg / 系统路径)
find_path(LASZIP_INCLUDE_DIR laszip/laszip_api.h
    PATHS /usr/include /usr/local/include /snap/cloudcompare/208/usr/include ${CMAKE_PREFIX_PATH}/include)
find_library(LASZIP_LIBRARY NAMES laszip laszip3
    PATHS /usr/lib /usr/local/lib /snap/cloudcompare/208/usr/lib ${CMAKE_PREFIX_PATH}/lib)

# 查找 yaml-cpp
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h)
    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

if(LASZIP_INCLUDE_DIR)
    include_directories(${LASZIP_INCLUDE_DIR})
endif()

if(YAML_CPP_INCLUDE_DIR)
    include_directories(${YAML_CPP_INCLUDE_DIR})
endif()

# 源文件匹配
file(GLOB_RECURSE SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cc")
# 过滤已移除的 Camera.cc
list(FILTER SRC_FILES EXCLUDE REGEX ".*Camera\\.cc$")

set(MAIN_SRC ${PROJECT_SOURCE_DIR}/test/main.cc)

add_executable(${PROJECT_NAME} 
    ${MAIN_SRC} 
    ${SRC_FILES}
)

# ===== 链接库 =====
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

if(LASZIP_LIBRARY)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LASZIP_LIBRARY})
endif()

if(TARGET yaml-cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
elseif(YAML_CPP_LIBRARY)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${YAML_CPP_LIBRARY})
else()
    # 尝试使用本地预编译静态库兜底
    add_library(yaml-cpp_static STATIC IMPORTED)
    if(WIN32)
        set_target_properties(yaml-cpp_static PROPERTIES IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/lib/yaml-cpp.lib")
    else()
        set_target_properties(yaml-cpp_static PROPERTIES IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/lib/libyaml-cpp.a")
    endif()
    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp_static)
endif()

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 setupapi)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()
