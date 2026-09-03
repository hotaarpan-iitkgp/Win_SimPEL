#pragma once

#include "engine/Components.hpp"
#include "ConfiguratorDialog.hpp"
#include "imgui.h"
#include <vector>
#include <string>
#include <set>
#include <stack>
#include <unordered_set>
#include <unordered_map>

namespace CircuitSim {

std::vector<TerminalDef> getTerminals(const ComponentInstance& comp);

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
    bool isDarkMode = true;
    
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

    // Adaptive box zoom
    bool adaptiveZoomMode = false;
    bool isBoxZooming = false;
    ImVec2 boxZoomStart{0.0f, 0.0f};
    ImVec2 boxZoomEnd{0.0f, 0.0f};
    
    // Dynamic Modals state
    bool showConfigurator = false;
    ComponentInstance pendingConfigComp;
    int pendingConfigCompIdx = -1;

    bool showCScriptModal = false;
    int cscriptCompIdx = -1;
    char cscriptCodeBuf[4096] = "";
    char cscriptTimestepBuf[64] = "0";

    bool showScopeModal = false;
    int scopeCompIdx = -1;
    char scopeChannelsBuf[32] = "2";

    bool showPulseModal = false;
    int pulseCompIdx = -1;
    char pulseAmpBuf[64] = "1";
    char pulsePeriodBuf[64] = "1";
    char pulseWidthBuf[64] = "0.5";
    char pulseDelayBuf[64] = "0";

    bool showCompConstModal = false;
    int compConstCompIdx = -1;
    int compConstOpIdx = 0;
    char compConstValBuf[64] = "0.0";

    bool showEdgeDetectModal = false;
    int edgeDetectCompIdx = -1;
    int edgeDetectTypeIdx = 0;
    char edgeDetectPulseWidthBuf[64] = "1e-3";

    bool showHitCrossingModal = false;
    int hitCrossingCompIdx = -1;
    int hitCrossingDirIdx = 0;
    char hitCrossingOffsetBuf[64] = "0.0";

    bool showRoundModal = false;
    int roundCompIdx = -1;
    int roundModeIdx = 0;

    bool showSineWaveModal = false;
    int sineWaveCompIdx = -1;
    char sineWaveAmpBuf[64] = "1.0";
    char sineWaveFreqBuf[64] = "50.0";
    char sineWavePhaseBuf[64] = "0.0";
    char sineWaveBiasBuf[64] = "0.0";

    void drawGrid(ImDrawList* drawList, ImVec2 canvasSize, ImVec2 canvasPos);
    void drawComponents(ImDrawList* drawList, ImVec2 canvasPos);
    void drawWires(ImDrawList* drawList, ImVec2 canvasPos);
    void drawBreadcrumbs(ImDrawList* drawList, ImVec2 canvasPos);
    void drawTerminals(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImVec2 mousePos, float& minPinDist);
    bool getTerminalPortStub(const ComponentInstance& comp, const std::string& terminalName, ImVec2 canvasPos, float zoomLevel, ImVec2& outPinPos, ImVec2& outStubPos, bool& outIsVertical) const;
    static void getComponentBounds(const ComponentInstance& comp, float& outHalfW, float& outHalfH);
    void renderModals();

    bool isPinConnected(const std::string& compId, const std::string& pinName) const;

    // ── Per-frame lookup caches ──────────────────────────────────────────────
    // Rebuilt once at the top of render(). Without these, drawTerminals() called
    // isPinConnected() for every pin of every component, and each of those scanned
    // every wire while allocating uppercase copies of terminal names inside
    // isTerminalMatch() — O(components x pins x wires) heap churn every frame.
    mutable std::unordered_set<std::string> frameConnectedPins;
    mutable std::unordered_map<std::string, std::vector<TerminalDef>> frameTerminals;
    mutable std::unordered_map<std::string, const ComponentInstance*> frameCompMap;
    mutable std::unordered_map<std::string, int> framePinDomain;
    void rebuildFrameCaches() const;
    const std::vector<TerminalDef>& cachedTerminals(const ComponentInstance& comp) const;
    const ComponentInstance* findComp(const std::string& id) const;
    // getPinDomain() builds two uppercase std::strings on every call; memoise it.
    // pinDomainIdCached returns the DomainType as an int: 0 = Power, 1 = Control,
    // 2 = Magnetic (DomainType itself is private to SchematicCanvas.cpp).
    int pinDomainIdCached(const ComponentInstance& comp, const std::string& pinName) const;
    bool isControlPinCached(const ComponentInstance& comp, const std::string& pinName) const;

    // Visible-area culling bounds (screen space), set each frame in render().
    mutable ImVec2 cullMin{0.0f, 0.0f};
    mutable ImVec2 cullMax{0.0f, 0.0f};
    bool isPointVisible(const ImVec2& p, float margin) const {
        return p.x >= cullMin.x - margin && p.x <= cullMax.x + margin &&
               p.y >= cullMin.y - margin && p.y <= cullMax.y + margin;
    }
    bool isSegmentVisible(const ImVec2& a, const ImVec2& b, float margin) const {
        float loX = (a.x < b.x ? a.x : b.x) - margin, hiX = (a.x > b.x ? a.x : b.x) + margin;
        float loY = (a.y < b.y ? a.y : b.y) - margin, hiY = (a.y > b.y ? a.y : b.y) + margin;
        return !(hiX < cullMin.x || loX > cullMax.x || hiY < cullMin.y || loY > cullMax.y);
    }

    // Zoom-aware label rendering. Text drawn through these scales with the canvas
    // instead of staying at a fixed pixel size, and disappears once it would be
    // too small to read (which also removes the overlap seen when zoomed out).
    static constexpr float LABEL_MIN_PX = 5.0f;
    void drawScaledText(ImDrawList* dl, const ImVec2& pos, ImU32 col, const char* text, float zoom, float basePx = 0.0f) const;
    ImVec2 calcScaledTextSize(const char* text, float zoom, float basePx = 0.0f) const;
    
    // Routing & Validation Engine (matching schematic_routing_guide.md)
    void autoSelectWiresForSelectedComponents();
    void deleteSelected();
    void cleanupOrphanedJunctions();
    void consolidateOverlappingWires();
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

    static void drawComponentShape(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImU32 color, bool isDarkMode = true);

    void setDarkMode(bool dark) { isDarkMode = dark; }
    bool isDarkModeActive() const { return isDarkMode; }

    void syncProbeSignals();

    void setCircuit(const CircuitDesign& circuit) { design = circuit; cleanupOrphanedJunctions(); consolidateOverlappingWires(); pushUndoState(); }
    const CircuitDesign& getCircuit() const { return design; }
    CircuitDesign& getCircuitRef() { return design; }

    void addComponent(const ComponentInstance& comp);
    void render(const char* title, ImVec2 size);

    mutable ImVec2 lastRenderedCanvasSize{800.0f, 600.0f};

    void copySelected();
    void pasteSelected();
    void duplicateSelected();
    void flipHorizontal();
    void flipVertical();
    void fitToScreen(ImVec2 canvasSize = ImVec2(0.0f, 0.0f));
    
    void undo();
    void redo();

    void openCScriptModalForComp(const std::string& compId);
    void autoConnectComponents(const ComponentInstance& targetComp);

    const std::set<std::string>& getSelectedComponentIds() const { return selectedComponentIds; }
    const std::set<std::string>& getSelectedWireIds() const { return selectedWireIds; }
    ComponentInstance* getSelectedComponent();
    std::vector<ComponentInstance*> getSelectedComponents();

    bool isAdaptiveZoomMode() const { return adaptiveZoomMode; }
    void toggleAdaptiveZoom() { adaptiveZoomMode = !adaptiveZoomMode; isBoxZooming = false; }

    // Scope window open request (set by double-click on SCOPE, consumed by MainWindow)
    struct ScopeOpenRequest {
        bool pending = false;
        std::string scopeId;
        int numChannels = 2;
    };
    ScopeOpenRequest scopeOpenRequest;
};

} // namespace CircuitSim
