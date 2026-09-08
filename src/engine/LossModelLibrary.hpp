#pragma once

#include "LossModel.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace CircuitSimEngine {

class LossModelLibrary {
public:
    static LossModelLibrary& getInstance() {
        static LossModelLibrary instance;
        return instance;
    }

    void loadAllModels(const std::string& base_dir);
    
    std::shared_ptr<LossModel> getModel(const std::string& part_number);
    std::vector<std::string> getAvailableModels() const;

private:
    LossModelLibrary() = default;
    
    std::map<std::string, std::shared_ptr<LossModel>> models;
    std::vector<std::string> model_names;
};

} // namespace CircuitSimEngine
