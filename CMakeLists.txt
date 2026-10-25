cmake_minimum_required(VERSION 3.16)
project(CircuitSimPro_Win LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SIMPEL_BUILD_GUI    "Build the ImGui desktop application" ON)
option(SIMPEL_BUILD_TESTS  "Build the regression tests" ON)
option(SIMPEL_BUILD_PYTHON "Build the simpel._core Python extension" OFF)

# MinGW builds link the runtime statically so the executables and the Python
# extension do not depend on libstdc++/libgcc/libwinpthread DLLs being on PATH.
set(SIMPEL_STATIC_RUNTIME "")
if(MINGW)
    set(SIMPEL_STATIC_RUNTIME -static -static-libgcc -static-libstdc++)
endif()

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/implot)

# ── simpel_core: the headless engine ────────────────────────────────────────
# Design I/O, netlist generation, parsing, solver and analyses. Must not depend on
# ImGui, ImPlot or any UI code: the CLI and the Python module link against it.
# Only src/ and the header-only nlohmann/json are on its include path, so a UI
# include fails to compile; the core_no_ui_deps test guards the rest.
set(SIMPEL_CORE_SOURCES
    src/engine/CircuitSimulator.cpp
    src/engine/NetlistParser.cpp
    src/engine/NetlistBuilder.cpp
    src/engine/NetlistGenerator.cpp
    src/engine/DesignNormalize.cpp
    src/engine/DesignIO.cpp
    src/engine/Runner.cpp
    src/engine/Circuit.cpp
    src/engine/SVGExporter.cpp
    src/engine/Exporters.cpp
    src/engine/Decimate.cpp
    src/engine/Sweep.cpp
    src/engine/layout/Placer.cpp
    src/engine/layout/Router.cpp
    src/engine/ResultsFile.cpp
    src/engine/Terminals.cpp
    src/engine/LossModel.cpp
    src/engine/LossModelLibrary.cpp
    src/engine/LossAnalysisEngine.cpp
    src/engine/CScriptEngine.cpp
    src/engine/ExpressionEvaluator.cpp
    src/engine/SignalAnalysis.cpp
    src/engine/FourierAnalysis.cpp
)
# nlohmann/json lives in 3rdparty/nlohmann/json.hpp. Expose only that header to the
# core, via a copy in the build tree, rather than all of 3rdparty (which also holds
# imgui and implot).
set(SIMPEL_JSON_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/json_include)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nlohmann/json.hpp
               ${SIMPEL_JSON_INCLUDE}/nlohmann/json.hpp COPYONLY)

# Generated sources: ComponentType name table and the embedded block registry data.
set(SIMPEL_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${SIMPEL_GEN_DIR})
add_custom_command(
    OUTPUT ${SIMPEL_GEN_DIR}/ComponentTypeNames.inc
    COMMAND ${CMAKE_COMMAND} -DIN=${CMAKE_CURRENT_SOURCE_DIR}/src/engine/Components.hpp
            -DOUT=${SIMPEL_GEN_DIR}/ComponentTypeNames.inc
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_component_type_names.cmake
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/engine/Components.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_component_type_names.cmake
    COMMENT "Generating ComponentType name table")
file(GLOB SIMPEL_BLOCK_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/data/blocks/*.json)
string(REPLACE ";" "\\;" SIMPEL_BLOCK_FILES_ARG "${SIMPEL_BLOCK_FILES}")
add_custom_command(
    OUTPUT ${SIMPEL_GEN_DIR}/EmbeddedBlocks.cpp
    COMMAND ${CMAKE_COMMAND} -DOUT=${SIMPEL_GEN_DIR}/EmbeddedBlocks.cpp -DSYMBOL=kBlockFiles
            "-DFILES=${SIMPEL_BLOCK_FILES_ARG}"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_files.cmake
    DEPENDS ${SIMPEL_BLOCK_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_files.cmake
    COMMENT "Embedding data/blocks/*.json")
list(APPEND SIMPEL_CORE_SOURCES
    src/engine/BlockRegistry.cpp
    ${SIMPEL_GEN_DIR}/ComponentTypeNames.inc
    ${SIMPEL_GEN_DIR}/EmbeddedBlocks.cpp)

find_package(Threads REQUIRED)
add_library(simpel_core STATIC ${SIMPEL_CORE_SOURCES})
target_link_libraries(simpel_core PUBLIC Threads::Threads)
target_include_directories(simpel_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SIMPEL_JSON_INCLUDE})
target_include_directories(simpel_core PRIVATE ${SIMPEL_GEN_DIR})
set_target_properties(simpel_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── simpel_cli: headless command line ───────────────────────────────────────
add_executable(simpel_cli src/cli/simpel_cli.cpp)
target_link_libraries(simpel_cli PRIVATE simpel_core)
target_link_options(simpel_cli PRIVATE ${SIMPEL_STATIC_RUNTIME})

# ── simpel_ui + GUI executable ──────────────────────────────────────────────
if(SIMPEL_BUILD_GUI)
    set(SIMPEL_UI_SOURCES
        src/ui/ConfiguratorDialog.cpp
        src/ui/SchematicCanvas.cpp
        src/ui/OscilloscopeView.cpp
        src/ui/ScopeWindow.cpp
        src/ui/TracePlotter.cpp
        src/ui/NetlistSourceView.cpp
        src/ui/SVGExporter.cpp
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
    add_library(simpel_ui STATIC ${SIMPEL_UI_SOURCES})
    target_include_directories(simpel_ui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
        ${IMGUI_DIR}
        ${IMPLOT_DIR})
    target_link_libraries(simpel_ui PUBLIC
        simpel_core
        opengl32
        gdi32
        user32
        shell32
        comdlg32
        dwmapi
        winmm
    )

    add_executable(circuitsim_pro_win src/main.cpp)
    target_link_libraries(circuitsim_pro_win PRIVATE simpel_ui)
    target_link_options(circuitsim_pro_win PRIVATE ${SIMPEL_STATIC_RUNTIME})
endif()

# ── Python extension: simpel._core ──────────────────────────────────────────
if(SIMPEL_BUILD_PYTHON)
    find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(_core MODULE python/csrc/simpel_core_module.cpp)
    target_link_libraries(_core PRIVATE simpel_core)
    if(MINGW)
        # CPython's pyconfig.h only defines MS_WIN64 for MSVC; without it GCC sees
        # the wrong Py_ssize_t / pointer-size configuration.
        target_compile_definitions(_core PRIVATE MS_WIN64)
        target_link_options(_core PRIVATE ${SIMPEL_STATIC_RUNTIME})
    endif()
    install(TARGETS _core LIBRARY DESTINATION simpel RUNTIME DESTINATION simpel)
    # Semiconductor loss models, so res.losses() works out of the box (~2.4 MB).
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/thermal_models DESTINATION simpel/data)
endif()

# ── Tests ───────────────────────────────────────────────────────────────────
if(SIMPEL_BUILD_TESTS)
    enable_testing()

    # Golden-output regression harness: bitwise digests of netlists and waveforms for
    # the designs in tests/golden/manifest.txt. Refresh with `golden_runner --update`.
    # Links simpel_core only, which also proves the core is usable without the UI.
    add_executable(golden_runner tests/golden/golden_runner.cpp)
    target_link_libraries(golden_runner PRIVATE simpel_core)
    target_link_options(golden_runner PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME golden
             COMMAND golden_runner --root ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_test(NAME core_no_ui_deps
             COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/engine
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_core_deps.cmake)

    add_executable(test_designio tests/test_designio.cpp)
    target_link_libraries(test_designio PRIVATE simpel_core)
    target_link_options(test_designio PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME designio COMMAND test_designio)

    add_executable(test_registry tests/registry/test_registry.cpp)
    target_link_libraries(test_registry PRIVATE simpel_core)
    target_link_options(test_registry PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME registry COMMAND test_registry ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_executable(test_emission tests/registry/test_emission.cpp)
    target_link_libraries(test_emission PRIVATE simpel_core)
    target_link_options(test_emission PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME emission
             COMMAND test_emission ${CMAKE_CURRENT_SOURCE_DIR}/tests/registry/emission_snapshot.txt)

    add_executable(test_custom_block tests/registry/test_custom_block.cpp)
    target_link_libraries(test_custom_block PRIVATE simpel_core)
    target_link_options(test_custom_block PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME custom_block COMMAND test_custom_block ${CMAKE_CURRENT_SOURCE_DIR}/data/blocks)

    add_executable(test_circuit tests/test_circuit.cpp)
    target_link_libraries(test_circuit PRIVATE simpel_core)
    target_link_options(test_circuit PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME circuit COMMAND test_circuit ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_executable(test_export tests/export/test_export.cpp)
    target_link_libraries(test_export PRIVATE simpel_core)
    target_link_options(test_export PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME export
             COMMAND test_export ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs
                     ${CMAKE_CURRENT_SOURCE_DIR}/tests/export/export_snapshot.txt)

    add_executable(test_sweep tests/test_sweep.cpp)
    target_link_libraries(test_sweep PRIVATE simpel_core)
    target_link_options(test_sweep PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME sweep COMMAND test_sweep ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_executable(test_layout tests/test_layout.cpp)
    target_link_libraries(test_layout PRIVATE simpel_core)
    target_link_options(test_layout PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME layout COMMAND test_layout ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_executable(test_route tests/test_route.cpp)
    target_link_libraries(test_route PRIVATE simpel_core)
    target_link_options(test_route PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME route COMMAND test_route ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_executable(test_resultsfile tests/test_resultsfile.cpp)
    target_link_libraries(test_resultsfile PRIVATE simpel_core)
    target_link_options(test_resultsfile PRIVATE ${SIMPEL_STATIC_RUNTIME})
    add_test(NAME resultsfile COMMAND test_resultsfile ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs)

    add_test(NAME cli_version COMMAND simpel_cli version)
    add_test(NAME cli_run_buck
             COMMAND simpel_cli run ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/designs/Buck_converter.json -t 1e-3)
endif()
