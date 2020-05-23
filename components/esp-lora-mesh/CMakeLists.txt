cmake_minimum_required(VERSION 3.5)

set(SOURCES loral1.cpp)
idf_component_register(SRCS ${SOURCES}
	INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE --std=c++17)
