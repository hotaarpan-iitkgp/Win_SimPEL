#pragma once

#include "Components.hpp"
#include "ExpressionEvaluator.hpp"
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

namespace CircuitSim {

struct TelemetryData {
    std::vector<double> timeHistory;
    std::unordered_map<std::string, std::vector<double>> voltages;
    std::unordered_map<std::string, std::vector<double>> currents;
    std::unordered_map<std::string, std::vector<double>> channelSignals;
};

class CircuitSimulator {
private:
    CircuitDesign design;
    SolverSettings settings;
    
    // Matrix dimensions & state
    int numNodes = 0;
    int numVoltageSources = 0;
    int numInductors = 0;
    int totalDim = 0;
    
    std::unordered_map<std::string, int> nodeToIdx;
    std::unordered_map<std::string, int> vSourceToIdx;
    std::unordered_map<std::string, int> inductorToIdx;
    
    // Dense Matrix LU Solver
    std::vector<double> M;
    std::vector<double> K;
    std::vector<double> B;
    std::vector<double> X;
    
    std::unordered_map<std::string, ScriptBlockEngine> scriptEngines;
    
    // Asynchronous solver thread state
    std::atomic<bool> isRunning{false};
    std::atomic<bool> isPaused{false};
    std::atomic<double> currentTime{0.0};
    
    TelemetryData telemetry;
    std::mutex telemetryMutex;

    void buildMNAMatrix();
    bool solveLU(int n, const std::vector<double>& A, const std::vector<double>& b, std::vector<double>& x);

public:
    CircuitSimulator() = default;
    ~CircuitSimulator() { stop(); }

    void loadCircuit(const CircuitDesign& circuit);
    void reset();
    void step();
    void runAsync();
    void pause();
    void resume();
    void stop();

    bool getIsRunning() const { return isRunning; }
    bool getIsPaused() const { return isPaused; }
    double getCurrentTime() const { return currentTime; }

    TelemetryData getTelemetryCopy();
};

} // namespace CircuitSim
