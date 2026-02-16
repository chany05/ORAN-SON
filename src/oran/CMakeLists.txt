find_library(ARMADILLO armadillo)
find_library(MLPACK mlpack)

set(mlpack_libraries)
if(NOT(${ARMADILLO} STREQUAL ARMADILLO-NOTFOUND) AND NOT(${MLPACK} STREQUAL MLPACK-NOTFOUND))
    set(mlpack_libraries ${ARMADILLO} ${MLPACK})
endif()

# ── LibTorch ──
set(LIBTORCH_DIR "$ENV{HOME}/libs/libtorch")

find_library(TORCH_LIB torch PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)
find_library(TORCH_CPU_LIB torch_cpu PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)
find_library(C10_LIB c10 PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)

if(NOT TORCH_LIB OR NOT TORCH_CPU_LIB OR NOT C10_LIB)
    message(FATAL_ERROR "LibTorch not found in ${LIBTORCH_DIR}/lib")
endif()

set(LIBTORCH_LIBRARIES ${TORCH_LIB} ${TORCH_CPU_LIB} ${C10_LIB})

# build_lib 이전에 include 경로 추가 → 오브젝트 타겟(liboran-obj)에도 적용됨
include_directories(SYSTEM
    "${LIBTORCH_DIR}/include"
    "${LIBTORCH_DIR}/include/torch/csrc/api/include"
)

# C++17 필수 (LibTorch 요구사항)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

build_lib(
  LIBNAME oran
  SOURCE_FILES
    model/PubSubInfra.cc
    model/E2AP.cc
    model/E2SM-KPM-measurements.cc
    model/ORAN-message-types.cc
    model/xAppHandover.cc
    model/xAppHandoverMlpackKmeans.cc
    model/xAppHandoverMaxRsrq.cc
    model/xAppHandoverSON.cc
  HEADER_FILES
    model/json.hpp
    model/PubSubInfra.h
    model/E2AP.h
    model/xApp.h
    model/E2SM-KPM-measurements.h
    model/E2SM-RC-control-types.h
    model/E2SM-RC-indication-types.h
    model/ORAN-message-types.h
    model/xAppHandover.h
    model/xAppHandoverMlpackKmeans.h
    model/xAppHandoverMaxRsrq.h
    model/xAppHandoverSON.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${mlpack_libraries}
    ${LIBTORCH_LIBRARIES}
)