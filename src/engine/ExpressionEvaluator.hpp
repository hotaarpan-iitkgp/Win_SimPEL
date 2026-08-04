#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cctype>

namespace CircuitSim {

class ExpressionEvaluator {
public:
    static double parseScientific(const std::string& str);
    static double evaluate(const std::string& expr, const std::unordered_map<std::string, double>& variables);
};

class ScriptBlockEngine {
public:
    std::unordered_map<std::string, double> stateVars;
    std::vector<double> inputs;
    std::vector<double> outputs;
    std::string scriptCode;

    void setScript(const std::string& code);
    void executeStep(double time, double dt, const std::vector<double>& inVals);
};

} // namespace CircuitSim
