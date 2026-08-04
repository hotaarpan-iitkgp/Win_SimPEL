#include "ExpressionEvaluator.hpp"
#include <iostream>

namespace CircuitSim {

double ExpressionEvaluator::parseScientific(const std::string& str) {
    if (str.empty()) return 0.0;
    
    std::string s = str;
    // Trim whitespace
    size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return 0.0;
    size_t last = s.find_last_not_of(" \t\r\n");
    s = s.substr(first, (last - first + 1));
    
    // Strip trailing unit names
    std::string cleanS;
    for (char c : s) {
        cleanS += (char)tolower(c);
    }
    
    // Check suffix multipliers
    double mult = 1.0;
    char lastC = s.back();
    if (lastC == 'p') { mult = 1e-12; s.pop_back(); }
    else if (lastC == 'n') { mult = 1e-9; s.pop_back(); }
    else if (lastC == 'u') { mult = 1e-6; s.pop_back(); }
    else if (lastC == 'm') { mult = 1e-3; s.pop_back(); }
    else if (lastC == 'k') { mult = 1e3; s.pop_back(); }
    else if (lastC == 'M') { mult = 1e6; s.pop_back(); }
    else if (lastC == 'G') { mult = 1e9; s.pop_back(); }
    
    try {
        double val = std::stod(s);
        return val * mult;
    } catch (...) {
        return 0.0;
    }
}

class Parser {
    std::string expr;
    size_t pos = 0;
    const std::unordered_map<std::string, double>& vars;

    void skipWhitespace() {
        while (pos < expr.length() && isspace(expr[pos])) pos++;
    }

    char peek() {
        skipWhitespace();
        return pos < expr.length() ? expr[pos] : '\0';
    }

    char get() {
        skipWhitespace();
        return pos < expr.length() ? expr[pos++] : '\0';
    }

public:
    Parser(const std::string& e, const std::unordered_map<std::string, double>& v) : expr(e), vars(v) {}

    double parseExpression() {
        return parseTernary();
    }

private:
    double parseTernary() {
        double cond = parseLogicalOr();
        if (peek() == '?') {
            get(); // skip '?'
            double trueVal = parseTernary();
            if (peek() == ':') get(); // skip ':'
            double falseVal = parseTernary();
            return (cond != 0.0) ? trueVal : falseVal;
        }
        return cond;
    }

    double parseLogicalOr() {
        double left = parseLogicalAnd();
        while (pos + 1 < expr.length() && expr[pos] == '|' && expr[pos + 1] == '|') {
            pos += 2;
            double right = parseLogicalAnd();
            left = (left != 0.0 || right != 0.0) ? 1.0 : 0.0;
        }
        return left;
    }

    double parseLogicalAnd() {
        double left = parseRelational();
        while (pos + 1 < expr.length() && expr[pos] == '&' && expr[pos + 1] == '&') {
            pos += 2;
            double right = parseRelational();
            left = (left != 0.0 && right != 0.0) ? 1.0 : 0.0;
        }
        return left;
    }

    double parseRelational() {
        double left = parseAddSub();
        skipWhitespace();
        if (pos + 1 < expr.length()) {
            std::string op2 = expr.substr(pos, 2);
            if (op2 == ">=") { pos += 2; return (left >= parseAddSub()) ? 1.0 : 0.0; }
            if (op2 == "<=") { pos += 2; return (left <= parseAddSub()) ? 1.0 : 0.0; }
            if (op2 == "==") { pos += 2; return (left == parseAddSub()) ? 1.0 : 0.0; }
            if (op2 == "!=") { pos += 2; return (left != parseAddSub()) ? 1.0 : 0.0; }
        }
        if (peek() == '>') { get(); return (left > parseAddSub()) ? 1.0 : 0.0; }
        if (peek() == '<') { get(); return (left < parseAddSub()) ? 1.0 : 0.0; }
        return left;
    }

    double parseAddSub() {
        double left = parseMulDiv();
        while (true) {
            char c = peek();
            if (c == '+') { get(); left += parseMulDiv(); }
            else if (c == '-') { get(); left -= parseMulDiv(); }
            else break;
        }
        return left;
    }

    double parseMulDiv() {
        double left = parsePrimary();
        while (true) {
            char c = peek();
            if (c == '*') { get(); left *= parsePrimary(); }
            else if (c == '/') {
                get();
                double right = parsePrimary();
                left = (right != 0.0) ? (left / right) : 0.0;
            }
            else break;
        }
        return left;
    }

    double parsePrimary() {
        char c = peek();
        if (c == '-') { get(); return -parsePrimary(); }
        if (c == '+') { get(); return parsePrimary(); }
        if (c == '!') { get(); return (parsePrimary() == 0.0) ? 1.0 : 0.0; }
        if (c == '(') {
            get();
            double val = parseExpression();
            if (peek() == ')') get();
            return val;
        }

        if (isdigit(c) || c == '.') {
            std::string numStr;
            while (pos < expr.length() && (isdigit(expr[pos]) || expr[pos] == '.' || expr[pos] == 'e' || expr[pos] == 'E')) {
                numStr += get();
            }
            try { return std::stod(numStr); } catch (...) { return 0.0; }
        }

        if (isalpha(c) || c == '_') {
            std::string name;
            while (pos < expr.length() && (isalnum(expr[pos]) || expr[pos] == '_' || expr[pos] == '[' || expr[pos] == ']')) {
                name += get();
            }

            // Function calls
            if (peek() == '(') {
                get(); // skip '('
                double arg1 = parseExpression();
                double arg2 = 0.0;
                if (peek() == ',') {
                    get();
                    arg2 = parseExpression();
                }
                if (peek() == ')') get();

                if (name == "sin") return std::sin(arg1);
                if (name == "cos") return std::cos(arg1);
                if (name == "tan") return std::tan(arg1);
                if (name == "asin") return std::asin(arg1);
                if (name == "acos") return std::acos(arg1);
                if (name == "atan") return std::atan(arg1);
                if (name == "abs" || name == "fabs") return std::fabs(arg1);
                if (name == "sqrt") return std::sqrt(arg1);
                if (name == "exp") return std::exp(arg1);
                if (name == "log") return std::log(arg1);
                if (name == "min") return std::min(arg1, arg2);
                if (name == "max") return std::max(arg1, arg2);
            }

            if (name == "PI" || name == "pi") return 3.14159265358979323846;

            auto it = vars.find(name);
            if (it != vars.end()) return it->second;
            return 0.0;
        }

        return 0.0;
    }
};

double ExpressionEvaluator::evaluate(const std::string& expr, const std::unordered_map<std::string, double>& variables) {
    if (expr.empty()) return 0.0;
    Parser p(expr, variables);
    return p.parseExpression();
}

void ScriptBlockEngine::setScript(const std::string& code) {
    scriptCode = code;
}

void ScriptBlockEngine::executeStep(double time, double dt, const std::vector<double>& inVals) {
    inputs = inVals;
    if (outputs.size() < 10) outputs.resize(10, 0.0);
    
    std::unordered_map<std::string, double> vars = stateVars;
    vars["time"] = time;
    vars["dt"] = dt;
    vars["PI"] = 3.14159265358979323846;
    for (size_t i = 0; i < inputs.size(); ++i) {
        vars["inputs[" + std::to_string(i) + "]"] = inputs[i];
        vars["In" + std::to_string(i + 1)] = inputs[i];
    }
    for (size_t i = 0; i < outputs.size(); ++i) {
        vars["outputs[" + std::to_string(i) + "]"] = outputs[i];
        vars["Out" + std::to_string(i + 1)] = outputs[i];
    }

    std::stringstream ss(scriptCode);
    std::string line;
    while (std::getline(ss, line)) {
        size_t commentIdx = line.find("//");
        if (commentIdx != std::string::npos) line = line.substr(0, commentIdx);
        
        size_t eqIdx = line.find("=");
        if (eqIdx != std::string::npos) {
            std::string lhs = line.substr(0, eqIdx);
            std::string rhs = line.substr(eqIdx + 1);
            
            // Cleanup lhs
            size_t sc = rhs.find(";");
            if (sc != std::string::npos) rhs = rhs.substr(0, sc);
            
            std::stringstream lhsSS(lhs);
            std::string token, varName;
            while (lhsSS >> token) {
                if (token != "double" && token != "const" && token != "float" && token != "int") {
                    varName = token;
                }
            }
            if (!varName.empty()) {
                double val = ExpressionEvaluator::evaluate(rhs, vars);
                vars[varName] = val;
                if (varName.rfind("outputs[", 0) == 0) {
                    int idx = std::stoi(varName.substr(8, varName.find("]") - 8));
                    if (idx >= 0 && idx < (int)outputs.size()) outputs[idx] = val;
                } else {
                    stateVars[varName] = val;
                }
            }
        }
    }
}

} // namespace CircuitSim
