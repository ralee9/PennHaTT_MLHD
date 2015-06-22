## CMakeList for /hatt_mlhd/src/


## Add libraries
# OpenGL
message(STATUS "FINDING OPENGL *************")
find_package(OpenGL REQUIRED)
message(STATUS "gl libs: " ${GL_LIBS})
include_directories(${OPENGL_INCLUDE_DIRS})
add_definitions(${OPENGL_DEFINITIONS})
message(STATUS "opengl_incl_dir: ${OPENGL_INCLUDE_DIRS} opengl_link_dir: ${OPENGL_LIBRARY_DIRS}")
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

# GLUT
message(STATUS "FINDING GLUT ****************")
find_package(GLUT REQUIRED)
message(STATUS "GLUT libs: " ${GLUT_LIBS})
include_directories(${GLUT_INCLUDE_DIRS})
add_definitions(${GLUT_DEFINITIONS})
message(STATUS "opengl_incl_dir: " ${GLUT_INCLUDE_DIRS} "opengl_link_dir: " ${GLUT_LIBRARY_DIRS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)


# Concatenate core libraries
set(CORELIBS ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
message(STATUS "corelibs: " ${CORELIBS})


# MLHD Library
set(ML_LIBS "${PROJECT_SOURCE_DIR}/lib/libmlhi_api_linux.a")


## Specify location for project header files/ dependencies
include_directories("${PROJECT_SOURCE_DIR}/include/")
include_directories("${PROJECT_SOURCE_DIR}/ml/")
include_directories("${PROJECT_SOURCE_DIR}/GL/")
include_directories("${PROJECT_SOURCE_DIR}/images/")
include_directories("${PROJECT_SOURCE_DIR}/build/XML/")


## Add include files to project
file(GLOB INCL_LIST
      "${PROJECT_SOURCE_DIR}/include/*.h"
      "${PROJECT_SOURCE_DIR}/include/*.hpp")

file(GLOB GL_HEADS
      "${PROJECT_SOURCE_DIR}/GL/*.h")

set(ML_HEADS
        "${PROJECT_SOURCE_DIR}/ml/ml_api.h"
        "${PROJECT_SOURCE_DIR}/ml/ml_error.h"
        "${PROJECT_SOURCE_DIR}/ml/ml_constants.h"
        "${PROJECT_SOURCE_DIR}/ml/ml_types.h")

## Add application data (images and XML files) dependencies & copy to build dir
# BMP images
file(GLOB IMAGES
        "${PROJECT_SOURCE_DIR}/images/*.bmp")

foreach(item ${IMAGES})
    # Obtain only the file name
    get_filename_component(file ${item} NAME)
    message(STATUS "copying ${file}...")
    # Copy files
    configure_file(${item} "${PROJECT_BINARY_DIR}/images/${file}" COPYONLY)
endforeach(item)

# Model XML files
file(GLOB XML
        "${PROJECT_SOURCE_DIR}/build/XML/*.xml")

foreach(item ${XML})
    # Obtain only the file name
    get_filename_component(file ${item} NAME)
    message(STATUS "copying ${file}...")
    # Copy files
    configure_file(${item} "${PROJECT_BINARY_DIR}/build/XML/${file}" COPYONLY)
endforeach(item)


## Tell CMake to create executable
add_executable(HaTT_MLHD
                "${PROJECT_SOURCE_DIR}/src/main_mlhd.cpp"
                "${PROJECT_SOURCE_DIR}/src/helper.cpp"
                "${PROJECT_SOURCE_DIR}/src/AccSynthHashMatrix.cpp"
                "${PROJECT_SOURCE_DIR}/src/pugixml.cpp"
                ${INCL_LIST}
                ${ML_HEADS}
                ${GL_HEADS}
                ${CORELIBS}
                ${ML_LIBS})

## Try to link directories after target is created
link_directories(${OPENGL_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})

## Link libraries to target
target_link_libraries(HaTT_MLHD ${CORELIBS} ${ML_LIBS})
