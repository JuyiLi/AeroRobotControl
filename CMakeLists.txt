﻿cmake_minimum_required(VERSION 3.5)
project(AeroRobotControl)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)

SET(INCLUDE_DIR 

    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/evpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/spdlog

)
include_directories(${INCLUDE_DIR})

SET(LIB_DIR 

    ${CMAKE_CURRENT_SOURCE_DIR}/lib
	
)
link_directories(${LIB_DIR})

SET(INCLUDE_FILES 

    ${CMAKE_CURRENT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}.h
)

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src/Clients/spdlog spdlog_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src/Clients CLIENTS_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src/Clients/evpp CLIENTS_EVPP_SRC)
add_library(Clients SHARED ${spdlog_SRC} ${CLIENTS_SRC} ${CLIENTS_EVPP_SRC})
target_include_directories(Clients PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/evpp
                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/spdlog)
target_compile_definitions(Clients PUBLIC SPDLOG_COMPILED_LIB)
target_link_libraries(Clients PUBLIC event pthread)

add_library(${CMAKE_PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/${CMAKE_PROJECT_NAME}.cpp ${INCLUDE_FILES})
target_link_libraries(${CMAKE_PROJECT_NAME} Clients pthread)

add_custom_target(copyfile ALL
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${CMAKE_PROJECT_NAME}.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${CMAKE_PROJECT_NAME}.so
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/build/libClients.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/libClients.so
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "copy head files to include diretory; copy lib files to lib directory"
)

add_dependencies(copyfile ${CMAKE_PROJECT_NAME})

SET(test_list

    test
    )
foreach(var ${test_list})
    add_executable(${CMAKE_PROJECT_NAME}_${var} ${CMAKE_CURRENT_SOURCE_DIR}/example/${var}.cpp)

    target_link_libraries(${CMAKE_PROJECT_NAME}_${var} ${CMAKE_PROJECT_NAME} Clients pthread)
    
endforeach()

