cmake_minimum_required(VERSION 3.14)
project(mpcc)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

execute_process(
  COMMAND python3 -c "import casadi,os;print(os.path.dirname(casadi.__file__))"
  OUTPUT_VARIABLE CASADI_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT EXISTS "${CASADI_DIR}/include/casadi/casadi.hpp")
  message(FATAL_ERROR "casadi not found. Activate the env where you ran pip install casadi.")
endif()

add_executable(mpcc main_mpcc.cpp mpcc_dynamic.cpp MPCC.cpp nlp_solve.cpp)
target_include_directories(mpcc PRIVATE ${CASADI_DIR}/include)
target_link_directories(mpcc PRIVATE ${CASADI_DIR})
target_link_libraries(mpcc PRIVATE casadi)
set_target_properties(mpcc PROPERTIES BUILD_RPATH ${CASADI_DIR})
# ---- Python module: import mpcc_py ----
execute_process(
  COMMAND python3 -m pybind11 --cmakedir
  OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
# Force CMake to use the ACTIVE conda env's Python (mujuco_sim), not the system Python 3.13.
# Mixing them gives "symbol not found ... _PyObject_ClearManagedDict" on import.
if(DEFINED ENV{CONDA_PREFIX})
  set(Python_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python" CACHE FILEPATH "" FORCE)
endif()
set(PYBIND11_FINDPYTHON ON)   # use modern FindPython, silences the CMP0148 warning
find_package(pybind11 REQUIRED)

pybind11_add_module(mpcc_py binding.cpp mpcc_controller.cpp MPCC.cpp mpcc_dynamic.cpp nlp_solve.cpp)
target_include_directories(mpcc_py PRIVATE ${CASADI_DIR}/include)
target_link_directories(mpcc_py PRIVATE ${CASADI_DIR})
target_link_libraries(mpcc_py PRIVATE casadi)
set_target_properties(mpcc_py PROPERTIES BUILD_RPATH ${CASADI_DIR})
