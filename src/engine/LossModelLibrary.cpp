#include "LossModelLibrary.hpp"
#include <filesystem>
#include <iostream>
#include <algorithm>

namespace fs = std::filesystem;

namespace CircuitSimEngine {

void LossModelLibrary::loadAllModels(const std::string& base_dir) {
    models.clear();
    model_names.clear();

    try {
        if (!fs::exists(base_dir)) return;

        for (const auto& entry : fs::recursive_directory_iterator(base_dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                auto model = std::make_shared<LossModel>(entry.path().string());
                if (model->isValid()) {
                    // Use filename as key
                    std::string key = entry.path().filename().string();
                    models[key] = model;
                    model_names.push_back(key);
                }
            }
        }

        std::sort(model_names.begin(), model_names.end());
    } catch (const std::exception& e) {
        std::cerr << "LossModelLibrary error: " << e.what() << "\n";
    }
}

std::shared_ptr<LossModel> LossModelLibrary::getModel(const std::string& key) {
    auto it = models.find(key);
    if (it != models.end()) return it->second;
    return nullptr;
}

std::vector<std::string> LossModelLibrary::getAvailableModels() const {
    return model_names;
}

} // namespace CircuitSimEngine
