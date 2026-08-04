#pragma once

#include "engine/Components.hpp"
#include "ConfiguratorDialog.hpp"
#include "imgui.h"
#include <vector>
#include <string>
#include <set>
#include <stack>

namespace CircuitSim {

struct SubsystemLevel {
    std::string name;
    CircuitDesign design;
};

class SchematicCanvas {
private:
    CircuitDesign design;
    
    // Subsystem Stack & Breadcrumb Navigation
    std::vector<SubsystemLevel> subsystemStack;

    // Undo / Redo History Stacks
    std::vector<CircuitDesign> undoStack;
    std::vector<CircuitDesign> redoStack;
    void pushUndoState();

    // Canvas pan & zoom state
    ImVec2 panOffset{400.0f, 400.0f};
    float zoomLevel = 1.0f;
    ImVec2 lastCanvasClickWorldPos{0.0f, 0.0f};
    bool hasLastClickPos = false;
    
    // Interactive selection & drag
    std::set<std::string> selectedComponentIds;
    std::set<std::string> selectedWireIds;
    
    // Wire Segment Dragging State
    bool isDraggingWireSegment = false;
    std::string draggingWireId;
    int draggingSegmentIdx = -1;
    bool isDraggingSegmentHorizontal = false;
    
    std::string hoveredPinCompId;
    std::string hoveredPinName;
    std::string hoveredWireId;
    ImVec2 hoveredWireJunctionPos{0.0f, 0.0f};
    
    // Wire creation mode
    bool isWiring = false;
    std::string wireStartCompId;
    std::string wireStartPin;
    ImVec2 wireCurrentPos{0.0f, 0.0f};
    std::vector<ImVec2> activeWireCorners;

    // Marquee box selection
    bool isBoxSelecting = false;
    ImVec2 boxSelectStart{0.0f, 0.0f};
    ImVec2 boxSelectEnd{0.0f, 0.0f};
    
    // Dynamic Modals state
    bool showConfigurator = false;
    ComponentInstance pendingConfigComp;
    int pendingConfigCompIdx = -1;

    bool showCScriptModal = false;
    int cscriptCompIdx = -1;
    char cscriptCodeBuf[4096] = "";

    bool showPulseModal = false;
    int pulseCompIdx = -1;
    char pulseFreqBuf[64] = "20000";
    char pulsePhaseBuf[64] = "0";
    char pulseDutyBuf[64] = "50";
    char pulseAmpBuf[64] = "1.0";

    void drawGrid(ImDrawList* drawList, ImVec2 canvasSize, ImVec2 canvasPos);
    void drawComponents(ImDrawList* drawList, ImVec2 canvasPos);
    void drawWires(ImDrawList* drawList, ImVec2 canvasPos);
    void drawBreadcrumbs(ImDrawList* drawList, ImVec2 canvasPos);
    void drawComponentShape(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImU32 color);
    void drawTerminals(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImVec2 mousePos, float& minPinDist);
    bool getTerminalPortStub(const ComponentInstance& comp, const std::string& terminalName, ImVec2 canvasPos, float zoomLevel, ImVec2& outPinPos, ImVec2& outStubPos, bool& outIsVertical) const;
    void renderModals();

    bool isPinConnected(const std::string& compId, const std::string& pinName) const;
    
    // Routing & Validation Engine (matching schematic_routing_guide.md)
    void normalizeControlWires();
    void rebuildNetlist();
    std::vector<ImVec2> simplifyPath(const std::vector<ImVec2>& points) const;
    bool validateSingleOutportConstraint(const std::string& startCompId, const std::string& startPin, const std::string& targetCompId, const std::string& targetPin) const;
    void drawCurrentFlowAnimation(ImDrawList* drawList, ImVec2 p1, ImVec2 mid, ImVec2 mid2, ImVec2 p2, bool isControlNet, float timeSec);

    // Coordinate helpers
    ImVec2 worldToScreen(float wx, float wy, ImVec2 canvasPos) const {
        return ImVec2(canvasPos.x + (wx + panOffset.x) * zoomLevel,
                      canvasPos.y + (wy + panOffset.y) * zoomLevel);
    }
    ImVec2 screenToWorld(ImVec2 screenPos, ImVec2 canvasPos) const {
        return ImVec2((screenPos.x - canvasPos.x) / zoomLevel - panOffset.x,
                      (screenPos.y - canvasPos.y) / zoomLevel - panOffset.y);
    }

public:
    SchematicCanvas() = default;

    void setCircuit(const CircuitDesign& circuit) { design = circuit; pushUndoState(); }
    const CircuitDesign& getCircuit() const { return design; }
    CircuitDesign& getCircuitRef() { return design; }

    void addComponent(const ComponentInstance& comp);
    void render(const char* title, ImVec2 size);

    void copySelected();
    void pasteSelected();
    void duplicateSelected();
    void flipHorizontal();
    void flipVertical();
    void fitToScreen(ImVec2 canvasSize);
    
    void undo();
    void redo();

    const std::set<std::string>& getSelectedComponentIds() const { return selectedComponentIds; }
    const std::set<std::string>& getSelectedWireIds() const { return selectedWireIds; }
    ComponentInstance* getSelectedComponent();
};

} // namespace CircuitSim
