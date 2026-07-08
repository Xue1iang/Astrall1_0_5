cmake_minimum_required(VERSION 3.10)
project(sdk_demo C CXX)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

file(GLOB SRC_FILE 
	"src/*.cpp"
)

include_directories(sdk_demo PRIVATE 
	src
	lib/include
)

add_executable(sdk_demo ${SRC_FILE})

target_link_libraries(sdk_demo 
        -pthread
	${CMAKE_SOURCE_DIR}/lib/libASTRALL_SDK.so
)
