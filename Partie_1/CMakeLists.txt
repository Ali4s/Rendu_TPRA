cmake_minimum_required(VERSION 3.5)
project(TERA)

set(CMAKE_CXX_STANDARD 14)

if (CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-value -Wno-unused-parameter")
elseif (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
endif ()

find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/png
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/opencv
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/opencv2
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/opencv2/features2d
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/glew/include
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/deps
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}

        ${OpenCV_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(extern/png PNG_SRC)
AUX_SOURCE_DIRECTORY(extern/glew/src GLEW_SRC)
AUX_SOURCE_DIRECTORY(extern/glfw/src GLFW_SRC)

set(
    SRC

    ${PNG_SRC}
    ${GLEW_SRC}
    ${GLFW_SRC}

    src/*.cpp
    include/*.hpp
    assets
)

# Defining preprocessor macros and selecting files to be removed
if (UNIX)
    add_definitions(
        -D_GLFW_X11
        -D_GLFW_HAS_XF86VM

        #-fsanitize=undefined
        #-fsanitize=address
        #-fsanitize=leak
    )

    file(
        GLOB
        DISPOSABLE

        extern/glfw/src/cocoa*
        extern/glfw/src/win32*
        extern/glfw/src/wl*
        extern/glfw/src/wgl*
        extern/glfw/src/mir*
        extern/glfw/src/*.m
    )

    set(
        LIBS

        #asan
        #ubsan
    )

    set(
        DEPS

        X11
        GL
        Xrandr
        Xcursor
        Xinerama
        Xxf86vm
        pthread
        dl
    )

endif ()

# Adding recursively every file we want to compile
file(
    GLOB
    SOURCE_FILES

    ${SRC}
)

# Removing unwanted platform-specific files
list(
    REMOVE_ITEM
    SOURCE_FILES

    ${DISPOSABLE}
)

add_executable(MJV-Projet ${SOURCE_FILES} main.cpp)

target_link_libraries(
    MJV-Projet ${LIBS}
    ${DEPS}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
)

