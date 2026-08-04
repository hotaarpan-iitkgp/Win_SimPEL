#pragma once
#include <string>
#include <unordered_map>

namespace CircuitSimEngine {

class ExpressionEvaluator {
public:
    static double parseScientific(const std::string& str);
    static double evaluateSimpleMath(const std::string& expr, const std::unordered_map<std::string, double>& vars = {});
    static double evaluate(const std::string& expr, const std::unordered_map<std::string, double>& vars = {});
};

} // namespace CircuitSimEngine
