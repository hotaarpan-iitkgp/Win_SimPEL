#include "OscilloscopeView.hpp"
#include <iostream>

namespace CircuitSim {

void OscilloscopeView::render(const char* title, CircuitSimulator& simulator) {
    ImGui::Begin(title);
    
    TelemetryData data = simulator.getTelemetryCopy();
    
    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No simulation waveform data available. Press PLAY to run simulation.");
        ImGui::End();
        return;
    }

    if (ImPlot::BeginPlot("Real-Time Oscilloscope Waveforms", ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Time (s)", "Voltage / Signal (V)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, simulator.getCurrentTime() + 1e-4, ImGuiCond_Always);
        
        for (const auto& pair : data.voltages) {
            const std::string& varName = pair.first;
            const std::vector<double>& vals = pair.second;
            
            int count = (int)std::min(data.timeHistory.size(), vals.size());
            if (count > 0) {
                ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count);
            }
        }

        ImPlot::EndPlot();
    }

    ImGui::End();
}

} // namespace CircuitSim
