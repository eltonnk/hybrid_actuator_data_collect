add_library(Csv csv.c csv.h)
add_library(Prbs prbs.c prbs.h)
add_library(LagBuffer lag_buffer.c lag_buffer.h)
add_library(LtiSys lti_sys.c lti_sys.h)
set_property(TARGET Csv PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set_property(TARGET Prbs PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set_property(TARGET LagBuffer PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set_property(TARGET LtiSys PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

add_executable(HybridActuatorDataCollect hyb_data_collect.c)
# This should be the CMake default but set it anyway:
# https://cmake.org/cmake/help/latest/prop_tgt/MSVC_RUNTIME_LIBRARY.html
set_property(TARGET HybridActuatorDataCollect PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
target_include_directories(HybridActuatorDataCollect PRIVATE . $ENV{HIL_DIR}/include)
target_link_directories(HybridActuatorDataCollect PRIVATE $ENV{HIL_DIR}/lib/win64)
target_link_libraries(HybridActuatorDataCollect 
    Csv LagBuffer LtiSys Prbs  
    hil quanser_runtime quanser_common legacy_stdio_definitions kernel32 user32
    gdi32 winspool comdlg32 advapi32 shell32 ole32 oleaut32 uuid odbc32
    odbccp32)