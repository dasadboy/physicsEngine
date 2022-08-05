set(BINARY ${CMAKE_PROJECT_NAME})

include_directories(include)

include_directories(src)

# .CPP FILES
set(SRC_FILE_NAMES 
    main.cpp
    collision/broadphase.cpp
    common/common.cpp)

list(TRANSFORM SRC_FILE_NAMES PREPEND "src/" OUTPUT_VARIABLE SRC_FILES)

# .H FILES

# set(COLLIDER_HEADER_FILE_NAMES
#     collider-base.h
#     capsule-collider.h 
#     plane-collider.h 
#     sphere-collider.h
#     collision-manager.h)

# list(TRANSFORM COLLIDER_HEADER_FILE_NAMES PREPEND "include/collision/" OUTPUT_VARIABLE COLLIDER_HEADERS)

# set(MATH_HEADER_FILE_NAMES 
#     math-defines.h 
#     algos.h 
#     vector3.h 
#     collision-vector.h 
#     transform.h 
#     basic-data-types.h)

# list(TRANSFORM MATH_HEADER_FILE_NAMES PREPEND "include/math/" OUTPUT_VARIABLE HEADER_FILES)

# list(APPEND HEADER_FILES ${COLLIDER_HEADERS})

file(GLOB HEADER_FILES include/*.h)

add_executable(${BINARY} ${HEADER_FILES} ${SRC_FILES})

add_library(${BINARY}-lib STATIC ${HEADER_FILES} ${SRC_FILES})
