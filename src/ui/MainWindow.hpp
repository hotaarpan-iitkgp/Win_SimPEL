#pragma once

#include "SchematicCanvas.hpp"
#include "OscilloscopeView.hpp"
#include "ScopeWindow.hpp"
#include "NetlistSourceView.hpp"
#include "SVGExporter.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

namespace CircuitSim {

enum class WorkspaceMode { SchematicCAD, WaveformNetlist };

class MainWindow {
private:
    SchematicCanvas canvas;
    OscilloscopeView scopeView;
    NetlistSourceView netlistSourceView;
    CircuitSimEngine::CircuitSimulator simulator;

    // Open scope popup windows (PLECS/MATLAB-style)
    std::vector<ScopeWindow> openScopeWindows;

    WorkspaceMode activeWorkspace = WorkspaceMode::SchematicCAD;
    std::string currentLoadedJsonName = "";
    bool isDarkMode = true;
    bool showComponentPalette = true;
    bool showDetailedLibrary = false;
    char searchPaletteBuf[128] = "";

    bool showDemoPane = true;
    char searchDemoBuf[128] = "";

    bool showSimParamsModal = false;
    char simStopTimeBuf[64] = "0.02";
    char simStepSizeBuf[64] = "1u";
    int simSolverIdx = 0;

    bool showCSCRIPTEditorModal = false;
    std::string editingCSCRIPTCompId = "";
    char cscriptCodeBuf[8192] = "";
    char cscriptTimestepBuf[64] = "0";

    // Report Export Modal State
    bool showExportOptionsModal = false;
    bool isBatchExportMode = false;
    std::string exportTargetFolder = "";
    std::string exportSingleFilePath = "";
    SVGExporter::ReportExportOptions currentExportOptions;

    // Background simulation thread
    std::thread simThread;
    std::atomic<bool> simRunning{false};

    void applyDarkTheme();
    void applyLightTheme();

    void renderMenuBar();
    void renderControlBar();
    void renderComponentPalette();
    void renderDemoPane();
    bool loadDemoJsonFile(const std::string& filename);
    void loadSchematicFromJson(const nlohmann::json& j);
    void renderPropertyInspector();
    void renderBatchPropertyInspector(const std::vector<ComponentInstance*>& selectedComps);
    void renderSimParamsModal();
    void renderCSCRIPTEditorModal();
    void renderExportOptionsModal();
    void openCSCRIPTEditor(const std::string& compId);

    void batchSimulateFolder(const std::string& folderPath);
    void batchExportHtmlFolder(const std::string& folderPath);
    void executeBatchExportWithOptions(const std::string& folderPath, const SVGExporter::ReportExportOptions& options);
    void executeSingleExportWithOptions(const SVGExporter::ReportExportOptions& options);

    // Scope window helpers
    void handleScopeOpenRequest();
    std::vector<std::string> traceScopeInputSignals(const std::string& scopeId, int numChannels);

public:
    MainWindow();
    ~MainWindow() { if (simThread.joinable()) simThread.join(); }

    void startSimulation();
    void loadPresetTemplate(const std::string& name);
    std::string getProjectBaseName() const;
    void render();
};

} // namespace CircuitSim
