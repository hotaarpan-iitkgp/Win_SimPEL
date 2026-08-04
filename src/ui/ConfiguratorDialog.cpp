#include "ConfiguratorDialog.hpp"
#include "imgui.h"

namespace CircuitSim {

bool ConfiguratorDialog::showConfiguratorModal(ComponentInstance& comp, bool* openFlag) {
    if (!openFlag || !(*openFlag)) return false;
    
    bool configured = false;
    ImGui::OpenPopup("Dynamic Component Configurator");
    
    if (ImGui::BeginPopupModal("Dynamic Component Configurator", openFlag, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Configuring: %s (%s)", comp.label.c_str(), comp.rawTypeStr.c_str());
        ImGui::Separator();
        ImGui::Spacing();
        
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "How many INPUT PINS do you need for this component? (e.g., 2, 3, 7, or 20?)");
        
        static int numPins = comp.numInputPins > 1 ? comp.numInputPins : 2;
        ImGui::InputInt("Input Pin Count", &numPins);
        if (numPins < 2) numPins = 2;
        if (numPins > 32) numPins = 32;

        ImGui::Spacing();
        ImGui::Text("Configure Operation Signs for each input pin:");
        
        static std::vector<char> signs;
        if (signs.size() != (size_t)numPins) {
            signs.assign(numPins, comp.type == ComponentType::Product ? '*' : '+');
        }

        for (int i = 0; i < numPins; ++i) {
            std::string label = "Pin " + std::to_string(i + 1) + " Sign";
            char currentSign = signs[i];
            if (comp.type == ComponentType::Product) {
                if (ImGui::RadioButton((label + " (*)").c_str(), currentSign == '*')) signs[i] = '*';
                ImGui::SameLine();
                if (ImGui::RadioButton((label + " (/)").c_str(), currentSign == '/')) signs[i] = '/';
            } else {
                if (ImGui::RadioButton((label + " (+)").c_str(), currentSign == '+')) signs[i] = '+';
                ImGui::SameLine();
                if (ImGui::RadioButton((label + " (-)").c_str(), currentSign == '-')) signs[i] = '-';
            }
        }

        ImGui::Spacing();
        if (comp.type != ComponentType::SummingJunction || comp.rawTypeStr != "SUM_ROUND") {
            ImGui::Checkbox("Include Top-Left CTRL Pin", &comp.hasCtrlPin);
        }

        ImGui::Separator();
        if (ImGui::Button("Apply Configuration", ImVec2(140, 30))) {
            comp.numInputPins = numPins;
            comp.pinSigns.clear();
            for (int i = 0; i < numPins; ++i) {
                comp.pinSigns.push_back(std::string(1, signs[i]));
            }
            
            // Dynamic body scaling based on pin count
            float spacing = 24.0f;
            comp.height = std::max(60.0f, (float)numPins * spacing + 20.0f);
            comp.width = 90.0f;
            
            // Re-generate pins
            comp.pins.clear();
            
            // Input pins on left
            for (int i = 0; i < numPins; ++i) {
                Pin p;
                p.name = "In" + std::to_string(i + 1);
                p.relativeX = 0.0f;
                p.relativeY = 15.0f + i * spacing;
                p.isInput = true;
                p.opSign = comp.pinSigns[i];
                comp.pins.push_back(p);
            }

            // Single output pin on right
            Pin outPin;
            outPin.name = "Out";
            outPin.relativeX = comp.width;
            outPin.relativeY = comp.height * 0.5f;
            outPin.isOutput = true;
            comp.pins.push_back(outPin);

            // Top-left CTRL pin for rectangular blocks
            if (comp.hasCtrlPin) {
                Pin ctrlPin;
                ctrlPin.name = "CTRL";
                ctrlPin.relativeX = 15.0f;
                ctrlPin.relativeY = 0.0f;
                ctrlPin.isCtrl = true;
                comp.pins.push_back(ctrlPin);
            }

            configured = true;
            *openFlag = false;
            ImGui::CloseCurrentPopup();
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            *openFlag = false;
            ImGui::CloseCurrentPopup();
        }
        
        ImGui::EndPopup();
    }
    
    return configured;
}

} // namespace CircuitSim
