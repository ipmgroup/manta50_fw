cmake_minimum_required(VERSION 3.13)

# Function:                 EXCLUDE_FILES_FROM_DIR_IN_LIST Description: Exclude
# all files from a list under a specific directory. Param _InFileList: Input and
# returned List Param _excludeDirName:    Name of the directory, which shall be
# ignored. Param _verbose:           Print the names of the files handled

# set(CMAKE_C_COMPILER_WORKS 1) set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-ti-c2000.cmake")
  message(STATUS "CMAKE_TOOLCHAIN_FILE not set, using default")
  include(${CMAKE_TOOLCHAIN_FILE})
endif()

set(PROJ_NAME "manta50")
set(BIN_SUFFIX ".out")
set(MAIN_BINARY "${PROJ_NAME}${BIN_SUFFIX}")

project(${PROJ_NAME} C ASM)

set(PROJECT_LOC ${CMAKE_CURRENT_SOURCE_DIR})

# set (EXCLUDE_DIR ${PROJECT_LOC}"include/libcanopen/include/arch")
set(EXCLUDE_DIRS
    # "${PROJECT_LOC}/include/libcanard"
    "${PROJECT_LOC}/include/libcanopen"
    "${PROJECT_LOC}/include/c2000-canopen"
    "${PROJECT_LOC}/include/libcanard/drivers/avr"
    "${PROJECT_LOC}/include/libcanard/drivers/linux"
    "${PROJECT_LOC}/include/libcanard/drivers/mcast"
    "${PROJECT_LOC}/include/libcanard/drivers/nuttx"
    "${PROJECT_LOC}/include/libcanard/drivers/socketcan"
    "${PROJECT_LOC}/include/libcanard/drivers/stm32"
    "${PROJECT_LOC}/include/libcanard/tests"
    "${PROJECT_LOC}/include/libcanard/examples"
    "${PROJECT_LOC}/include/libcanard/canard"
    )
# message("EXCLUDE_DIR: ${EXCLUDE_DIR}")

file(GLOB_RECURSE MODULE_SOURCES CONFIGURE_DEPENDS "include/*.c"
     "include/*.cpp")
file(GLOB_RECURSE MODULE_HEADERS CONFIGURE_DEPENDS "include/*.h")

set(MODULE_INCLUDE_DIRS "")
foreach(_headerFile ${MODULE_HEADERS})
  get_filename_component(_dir ${_headerFile} PATH)
  list(APPEND MODULE_INCLUDE_DIRS ${_dir})
endforeach()

list(REMOVE_DUPLICATES MODULE_INCLUDE_DIRS)

# message("EXCLUDE_DIRS: ${EXCLUDE_DIRS}")

foreach(EXCLUDE_DIR ${EXCLUDE_DIRS})
  list(FILTER MODULE_SOURCES EXCLUDE REGEX "^${EXCLUDE_DIR}/")
  list(FILTER MODULE_HEADERS EXCLUDE REGEX "^${EXCLUDE_DIR}/")
  list(FILTER MODULE_INCLUDE_DIRS EXCLUDE REGEX "^${EXCLUDE_DIR}/")
  list(REMOVE_ITEM MODULE_SOURCES${EXCLUDE_DIR})
  list(REMOVE_ITEM MODULE_HEADERS ${EXCLUDE_DIR})
  list(REMOVE_ITEM MODULE_INCLUDE_DIRS ${EXCLUDE_DIR})
endforeach()

include(mw-src)
write_file(SOURCES.txt "${MODULE_SOURCES}")
write_file(HEADERS.txt "${MODULE_HEADERS}")
write_file(MW_SRC.txt "${MW_SRC}")
# message("${MODULE_SOURCES}") message("${MODULE_HEADERS}")

# message("MODULE_INCLUDE_DIRS: ${MODULE_INCLUDE_DIRS}")

add_executable(${PROJ_NAME} ${MODULE_SOURCES} ${MW_SRC} manta50.c)
target_include_directories(
  ${PROJ_NAME} PRIVATE ${PROJECT_LOC} ${CG_TOOL_ROOT}/include ${MW_INSTALL_DIR}
                       ${MODULE_INCLUDE_DIRS})
add_compile_options(-Wall -c99)
target_compile_options(
  ${PROJ_NAME} BEFORE
  PUBLIC -v28
         -ml
         -mt
         -O2
         --abi=coffabi
         --diag_warning=225
         --display_error_number)
target_compile_definitions(
  ${PROJ_NAME}
  PUBLIC FLASH
         RESET_MCP_ON_INACTIVITY=1
         EEPROM_25AA02
         NODE_ID=0
         ARCH_BYTE16
         DRV8305
         CS_GPIO
         FAST_ROM_V1p7
         F2802xF
         CS_GPIO
         MCP2515
         nFault
         LED_ON)

target_link_directories(
  ${PROJ_NAME}
  PRIVATE
  ${PROJECT_LOC}
  ${CG_TOOL_ROOT}/lib
  ${CG_TOOL_ROOT}/include
  ${MW_INSTALL_DIR}/sw/modules/fast/lib/32b/f28x/f2802x/
  ${MW_INSTALL_DIR}/sw/modules/iqmath/lib/f28x/32b/)
target_link_options(
  ${PROJ_NAME}
  BEFORE
  PUBLIC
  --stack_size=0x180
  --warn_sections
  --diag_warning=225
  --display_error_number
  --diag_suppress=10063
  --priority
  --reread_libs
  --disable_auto_rts
  --entry_point=code_start
  --rom_model
  -o
  "${MAIN_BINARY}")
target_link_libraries(
  ${PROJ_NAME}
  -l2802xRevB_Fast_IQ_ROMSymbols.lib
  -l2802xRevB_Fast_ONLY_ROMSymbols.lib
  -l2802xRevB_Fast_RTS_ROMSymbols.lib
  -lfast_public.lib
  -lrts2800_ml.lib
  -lIQmath.lib
  -llibc.a
  ${PROJECT_LOC}/F28027F_can.cmd)

# Add command to flash device
if(EXISTS "${UNIFLASH_PATH}" AND EXISTS "${TARGET_CONFIG}")
  get_filename_component(UNIFLASH_EXT ${UNIFLASH_PATH} EXT)
  string(TOLOWER "${UNIFLASH_EXT}" UNIFLASH_EXT)
  if("${UNIFLASH_EXT}" STREQUAL ".sh")
    set(FLASH_CMD "")
  else()
    set(FLASH_CMD "flash")
  endif()
  add_custom_target(
    flash
    COMMAND ${UNIFLASH_PATH} ${FLASH_CMD} "--flash" "--verbose" "--verify"
            "--config=${TARGET_CONFIG}" "${CMAKE_BINARY_DIR}/${MAIN_BINARY}"
    DEPENDS "${CMAKE_BINARY_DIR}/${MAIN_BINARY}")
endif()
