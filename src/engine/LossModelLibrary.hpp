#pragma once

#include "LossModel.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

namespace CircuitSimEngine {

class LossModelLibrary {
public:
    static LossModelLibrary& getInstance() {
        static LossModelLibrary instance;
        return instance;
    }

    ~LossModelLibrary() {
        if (workerThread.joinable()) {
            workerThread.join();
        }
    }

    // Synchronous loading. Besides custom_dir and the built-in search locations
    // (next to the executable, the working directory and its parent), the
    // directories in SIMPEL_LOSS_MODELS_DIR (';'-separated) are scanned first.
    void loadAllModels(const std::string& custom_dir = "");
    // Replaces the library with the models found in exactly these directories
    // (searched recursively), ignoring the built-in search locations.
    void loadFromDirectories(const std::vector<std::string>& dirs);
    // Progress messages on stdout (default on, as the GUI shows them in its console).
    void setVerbose(bool v) { verbose.store(v); }
    void refresh();

    // Asynchronous loading (parallel background task)
    void loadAllModelsAsync(const std::string& custom_dir = "");
    void refreshAsync();

    bool isLoading() const { return loading.load(); }
    size_t getLoadedCount() const {
        std::lock_guard<std::mutex> lock(libraryMutex);
        return model_names.size();
    }
    
    std::shared_ptr<LossModel> getModel(const std::string& part_number);
    std::vector<std::string> getAvailableModels() const;
    std::vector<std::string> getScannedDirectories() const;

private:
    LossModelLibrary() = default;
    LossModelLibrary(const LossModelLibrary&) = delete;
    LossModelLibrary& operator=(const LossModelLibrary&) = delete;

    void scanDirectoryInternal(const std::string& dirPath);
    void loadAllModelsInternal(const std::string& custom_dir);

    mutable std::mutex libraryMutex;
    std::atomic<bool> loading{false};
    std::atomic<bool> verbose{true};
    std::thread workerThread;

    std::map<std::string, std::shared_ptr<LossModel>> models;
    std::vector<std::string> model_names;
    std::vector<std::string> loaded_dirs;
};

} // namespace CircuitSimEngine
