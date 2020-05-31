cmake_minimum_required(VERSION 3.5)

set(SOURCES lora_tdm.cpp lora_driver.cpp protocol.cpp)
idf_component_register(SRCS ${SOURCES}
	INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE --std=c++17)

if(DEFINED SLOT)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE TDM_THIS_SLOT_ID=${SLOT})
endif()
