# FindORB_SLAM3.cmake
# Find ORB_SLAM3 library and header files

set(ORB_SLAM3_ROOT_DIR "~/Documents/Projects/ORB_SLAM_COMMUNITY" CACHE PATH "ORB_SLAM3 root directory")

# Find header files
find_path(ORB_SLAM3_INCLUDE_DIR
    NAMES System.h
    PATHS ${ORB_SLAM3_ROOT_DIR}/include
    NO_DEFAULT_PATH
)

# Find main library
find_library(ORB_SLAM3_LIBRARY
    NAMES ORB_SLAM3
    PATHS ${ORB_SLAM3_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

# Find DBoW2 library
find_library(DBoW2_LIBRARY
    NAMES DBoW2
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
    NO_DEFAULT_PATH
)

# Find g2o library
find_library(g2o_LIBRARY
    NAMES g2o
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
    NO_DEFAULT_PATH
)

# Find yolov8 library
find_library(YOLOV8_LIBRARY
    NAMES yolov8_trt
    PATHS ${ORB_SLAM3_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

# Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

# YOLOv8 依赖
set(YOLOV8_DIR "${ORB_SLAM3_ROOT_DIR}/yolov8_trt")
set(ONNXRUNTIME_DIR "$ENV{HOME}/Documents/Projects/onnxruntime-1.17.3")

find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    PATHS ${ONNXRUNTIME_DIR}/lib
    NO_DEFAULT_PATH
)

# Set include directories
set(ORB_SLAM3_INCLUDE_DIRS
    ${ORB_SLAM3_ROOT_DIR}
    ${ORB_SLAM3_INCLUDE_DIR}
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${YOLOV8_DIR}/include
    ${ONNXRUNTIME_DIR}/include
)

# Set library list
set(ORB_SLAM3_LIBRARIES
    ${ORB_SLAM3_LIBRARY}
    ${DBoW2_LIBRARY}
    ${g2o_LIBRARY}
    ${YOLOV8_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${ONNXRUNTIME_LIBRARY}
    crypto
)

# Handle standard arguments and set ORB_SLAM3_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORB_SLAM3
    REQUIRED_VARS
        ORB_SLAM3_LIBRARY
        ORB_SLAM3_INCLUDE_DIR
        DBoW2_LIBRARY
        g2o_LIBRARY
        YOLOV8_LIBRARY
        ONNXRUNTIME_LIBRARY
    FOUND_VAR ORB_SLAM3_FOUND
)

mark_as_advanced(
    ORB_SLAM3_INCLUDE_DIR
    ORB_SLAM3_LIBRARY
    DBoW2_LIBRARY
    g2o_LIBRARY
    YOLOV8_LIBRARY
    ONNXRUNTIME_LIBRARY
)

# Create imported target (optional, but recommended)
if(ORB_SLAM3_FOUND AND NOT TARGET ORB_SLAM3::ORB_SLAM3)
    add_library(ORB_SLAM3::ORB_SLAM3 UNKNOWN IMPORTED)
    set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
        IMPORTED_LOCATION "${ORB_SLAM3_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ORB_SLAM3_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${ORB_SLAM3_LIBRARIES}"
    )
endif()
