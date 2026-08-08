cmake_minimum_required(VERSION 3.16)
project(CircuitSimPro_Win LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/implot)

include_directories(
    src
    3rdparty
    ${IMGUI_DIR}
    ${IMPLOT_DIR}
)

set(SOURCES
    src/main.cpp
    src/engine/CircuitSimulator.cpp
    src/engine/NetlistParser.cpp
    src/engine/CScriptEngine.cpp
    src/engine/ExpressionEvaluator.cpp
    src/engine/NetlistBuilder.cpp
    src/engine/SignalAnalysis.cpp
    src/ui/ConfiguratorDialog.cpp
    src/ui/SchematicCanvas.cpp
    src/ui/OscilloscopeView.cpp
    src/ui/ScopeWindow.cpp
    src/ui/NetlistSourceView.cpp
    src/ui/MainWindow.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_impl_win32.cpp
    ${IMGUI_DIR}/imgui_impl_opengl3.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

add_executable(circuitsim_pro_win ${SOURCES})

target_link_options(circuitsim_pro_win PRIVATE -static -static-libgcc -static-libstdc++)

target_link_libraries(circuitsim_pro_win
    opengl32
    gdi32
    user32
    shell32
    comdlg32
    dwmapi
    winmm
)

set_target_properties(circuitsim_pro_win PROPERTIES
    CXX_OPTIMIZE_OPTION "-O2"
)
