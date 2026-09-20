#include "LossModelLibrary.hpp"
#include <filesystem>
#include <iostream>
#include <algorithm>

#ifdef _WIN32
#include <windows.h>
#endif

namespace fs = std::filesystem;

namespace CircuitSimEngine {

static std::string getExecutableDir() {
#ifdef _WIN32
    char buffer[MAX_PATH];
    DWORD len = GetModuleFileNameA(NULL, buffer, MAX_PATH);
    if (len > 0) {
        return fs::path(buffer).parent_path().string();
    }
#endif
    return fs::current_path().string();
}

void LossModelLibrary::scanDirectoryInternal(const std::string& dirPath) {
    try {
        if (dirPath.empty()) return;
        std::error_code ec;
        if (!fs::exists(dirPath, ec) || ec) return;

        std::string canonicalPath = fs::canonical(dirPath, ec).string();
        if (ec) return;

        {
            std::lock_guard<std::mutex> lock(libraryMutex);
            for (const auto& d : loaded_dirs) {
                if (d == canonicalPath) return; // already scanned
            }
            loaded_dirs.push_back(canonicalPath);
        }

        std::cout << "[LossModelLibrary] Scanning (background): " << canonicalPath << std::endl;

        for (const auto& entry : fs::recursive_directory_iterator(dirPath, fs::directory_options::skip_permission_denied, ec)) {
            if (ec) break;
            if (entry.is_regular_file(ec) && entry.path().extension() == ".json") {
                std::string key = entry.path().filename().string();
                
                bool alreadyLoaded = false;
                {
                    std::lock_guard<std::mutex> lock(libraryMutex);
                    alreadyLoaded = (models.find(key) != models.end());
                }
                if (alreadyLoaded) continue;

                // Parse LossModel outside lock so file reading / JSON parsing runs in background without blocking UI
                auto model = std::make_shared<LossModel>(entry.path().string());
                if (model->isValid()) {
                    std::lock_guard<std::mutex> lock(libraryMutex);
                    if (models.find(key) == models.end()) {
                        models[key] = model;
                        model_names.push_back(key);
                        std::sort(model_names.begin(), model_names.end());
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[LossModelLibrary] Error scanning " << dirPath << ": " << e.what() << std::endl;
    }
}

void LossModelLibrary::loadAllModelsInternal(const std::string& custom_dir) {
    loading = true;

    std::string exeDir = getExecutableDir();
    std::vector<std::string> candidateDirs;

    // 1. Inside simulation tool executable directory: folder named 'loss_models'
    candidateDirs.push_back((fs::path(exeDir) / "loss_models").string());
    candidateDirs.push_back((fs::path(exeDir) / "data" / "thermal_models").string());

    // 2. Custom directory if explicitly requested
    if (!custom_dir.empty()) {
        candidateDirs.push_back(custom_dir);
    }

    // 3. Fallbacks relative to current working directory & parent project directory
    candidateDirs.push_back((fs::current_path() / "loss_models").string());
    candidateDirs.push_back((fs::current_path() / "build" / "loss_models").string());
    candidateDirs.push_back((fs::current_path() / "data" / "thermal_models").string());
    candidateDirs.push_back((fs::current_path() / ".." / "data" / "thermal_models").string());
    candidateDirs.push_back((fs::current_path() / ".." / "build" / "loss_models").string());

    for (const auto& dir : candidateDirs) {
        scanDirectoryInternal(dir);
    }

    {
        std::lock_guard<std::mutex> lock(libraryMutex);
        std::sort(model_names.begin(), model_names.end());
        model_names.erase(std::unique(model_names.begin(), model_names.end()), model_names.end());
    }

    std::cout << "[LossModelLibrary] Background loading complete. Total models: " 
              << getLoadedCount() << std::endl;
    loading = false;
}

void LossModelLibrary::loadAllModels(const std::string& custom_dir) {
    if (workerThread.joinable()) {
        workerThread.join();
    }
    {
        std::lock_guard<std::mutex> lock(libraryMutex);
        models.clear();
        model_names.clear();
        loaded_dirs.clear();
    }
    loadAllModelsInternal(custom_dir);
}

void LossModelLibrary::loadAllModelsAsync(const std::string& custom_dir) {
    if (loading.load()) {
        return; // Already in progress
    }
    if (workerThread.joinable()) {
        workerThread.join();
    }
    loading = true;
    workerThread = std::thread([this, custom_dir]() {
        loadAllModelsInternal(custom_dir);
    });
}

void LossModelLibrary::refresh() {
    loadAllModels("");
}

void LossModelLibrary::refreshAsync() {
    if (loading.load()) {
        return;
    }
    if (workerThread.joinable()) {
        workerThread.join();
    }
    {
        std::lock_guard<std::mutex> lock(libraryMutex);
        models.clear();
        model_names.clear();
        loaded_dirs.clear();
    }
    loadAllModelsAsync("");
}

std::shared_ptr<LossModel> LossModelLibrary::getModel(const std::string& key) {
    std::lock_guard<std::mutex> lock(libraryMutex);
    auto it = models.find(key);
    if (it != models.end()) return it->second;
    return nullptr;
}

std::vector<std::string> LossModelLibrary::getAvailableModels() const {
    std::lock_guard<std::mutex> lock(libraryMutex);
    return model_names;
}

std::vector<std::string> LossModelLibrary::getScannedDirectories() const {
    std::lock_guard<std::mutex> lock(libraryMutex);
    return loaded_dirs;
}

} // namespace CircuitSimEngine
