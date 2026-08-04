#include "ExpressionEvaluator.hpp"
#include <sstream>
#include <cctype>
#include <cmath>
#include <algorithm>
#include <vector>
#include <stack>
#include <iostream>

namespace CircuitSimEngine {

double ExpressionEvaluator::parseScientific(const std::string& str) {
    if (str.empty()) return 0.0;
    
    std::string s = str;
    // Trim whitespace
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
    if (s.empty()) return 0.0;

    if (s.length() >= 2 && (s.substr(s.length() - 2) == "Hz" || s.substr(s.length() - 2) == "hz")) {
        s.pop_back();
        s.pop_back();
        return parseScientific(s);
    }

    // Check for metric suffixes at the end
    double multiplier = 1.0;
    char last = s.back();

    if (last == 'k' || last == 'K') { multiplier = 1e3; s.pop_back(); }
    else if (last == 'M') { multiplier = 1e6; s.pop_back(); }
    else if (last == 'G' || last == 'g') { multiplier = 1e9; s.pop_back(); }
    else if (last == 'm') { multiplier = 1e-3; s.pop_back(); }
    else if (last == 'u' || last == 'U') { multiplier = 1e-6; s.pop_back(); }
    else if (last == 'n' || last == 'N') { multiplier = 1e-9; s.pop_back(); }
    else if (last == 'p' || last == 'P') { multiplier = 1e-12; s.pop_back(); }
    else if (last == 'f' || last == 'F') { multiplier = 1e-15; s.pop_back(); }
    else if (last == 'V' || last == 'v' || last == 'A' || last == 'a' || last == 'O' || last == 'o') {
        s.pop_back();
        return parseScientific(s);
    }

    try {
        size_t idx = 0;
        double val = std::stod(s, &idx);
        return val * multiplier;
    } catch (...) {
        return 0.0;
    }
}

double ExpressionEvaluator::evaluateSimpleMath(const std::string& expr, const std::unordered_map<std::string, double>& vars) {
    return evaluate(expr, vars);
}

static int precedence(const std::string& op) {
    if (op == "||") return 1;
    if (op == "&&") return 2;
    if (op == "==" || op == "!=") return 3;
    if (op == "<" || op == ">" || op == "<=" || op == ">=") return 4;
    if (op == "+" || op == "-") return 5;
    if (op == "*" || op == "/" || op == "%") return 6;
    if (op == "UNARY") return 7;
    return 0;
}

static bool isOperator(const std::string& op) {
    return (op == "+" || op == "-" || op == "*" || op == "/" || op == "%" ||
            op == "<" || op == ">" || op == "<=" || op == ">=" || op == "==" || op == "!=" ||
            op == "&&" || op == "||");
}

double ExpressionEvaluator::evaluate(const std::string& exprStr, const std::unordered_map<std::string, double>& vars) {
    std::string s = exprStr;
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
    if (s.empty()) return 0.0;

    // Direct variable lookup
    auto it = vars.find(s);
    if (it != vars.end()) return it->second;

    // Tokenize
    std::vector<std::string> tokens;
    size_t i = 0;
    while (i < s.size()) {
        if (std::isspace(s[i])) { i++; continue; }

        if (s[i] == '+' || s[i] == '-' || s[i] == '*' || s[i] == '/' || s[i] == '%' || s[i] == '(' || s[i] == ')') {
            // Check for 2-char comparison operators
            if (i + 1 < s.size()) {
                std::string sub2 = s.substr(i, 2);
                if (sub2 == "==" || sub2 == "!=" || sub2 == "<=" || sub2 == ">=" || sub2 == "&&" || sub2 == "||") {
                    tokens.push_back(sub2);
                    i += 2;
                    continue;
                }
            }
            tokens.push_back(std::string(1, s[i]));
            i++;
        } else if (s[i] == '<' || s[i] == '>' || s[i] == '!') {
            if (i + 1 < s.size() && s[i + 1] == '=') {
                tokens.push_back(s.substr(i, 2));
                i += 2;
            } else {
                tokens.push_back(std::string(1, s[i]));
                i++;
            }
        } else if (s[i] == '&' && i + 1 < s.size() && s[i + 1] == '&') {
            tokens.push_back("&&");
            i += 2;
        } else if (s[i] == '|' && i + 1 < s.size() && s[i + 1] == '|') {
            tokens.push_back("||");
            i += 2;
        } else {
            // Identifier, variable, function, array indexing inputs[0], or numeric literal
            size_t start = i;
            int bracketDepth = 0;
            while (i < s.size()) {
                if (s[i] == '[') bracketDepth++;
                else if (s[i] == ']') bracketDepth--;
                else if (bracketDepth == 0 && (std::isspace(s[i]) || s[i] == '+' || s[i] == '-' || s[i] == '*' || s[i] == '/' || s[i] == '%' || s[i] == '(' || s[i] == ')' || s[i] == '<' || s[i] == '>' || s[i] == '=' || s[i] == '!' || s[i] == '&' || s[i] == '|' || s[i] == '?')) {
                    break;
                }
                i++;
            }
            tokens.push_back(s.substr(start, i - start));
        }
    }

    // Shunting-Yard algorithm to evaluate infix tokens
    std::stack<double> valStack;
    std::stack<std::string> opStack;

    auto applyOp = [](const std::string& op, double a, double b = 0.0) -> double {
        if (op == "UNARY-") return -a;
        if (op == "UNARY+") return +a;
        if (op == "+") return a + b;
        if (op == "-") return a - b;
        if (op == "*") return a * b;
        if (op == "/") return (std::abs(b) > 1e-15) ? (a / b) : 0.0;
        if (op == "%") return (b != 0) ? std::fmod(a, b) : 0.0;
        if (op == "<") return (a < b) ? 1.0 : 0.0;
        if (op == ">") return (a > b) ? 1.0 : 0.0;
        if (op == "<=") return (a <= b) ? 1.0 : 0.0;
        if (op == ">=") return (a >= b) ? 1.0 : 0.0;
        if (op == "==") return (std::abs(a - b) < 1e-9) ? 1.0 : 0.0;
        if (op == "!=") return (std::abs(a - b) >= 1e-9) ? 1.0 : 0.0;
        if (op == "&&") return (a != 0.0 && b != 0.0) ? 1.0 : 0.0;
        if (op == "||") return (a != 0.0 || b != 0.0) ? 1.0 : 0.0;
        return 0.0;
    };

    bool expectUnary = true;

    for (size_t k = 0; k < tokens.size(); ++k) {
        std::string tok = tokens[k];

        if (tok == "(") {
            opStack.push(tok);
            expectUnary = true;
        } else if (tok == ")") {
            while (!opStack.empty() && opStack.top() != "(") {
                std::string op = opStack.top();
                opStack.pop();

                if (op == "UNARY-" || op == "UNARY+") {
                    if (!valStack.empty()) {
                        double v = valStack.top(); valStack.pop();
                        valStack.push(applyOp(op, v));
                    }
                } else if (op == "sin" || op == "cos" || op == "abs" || op == "sqrt") {
                    if (!valStack.empty()) {
                        double v = valStack.top(); valStack.pop();
                        if (op == "sin") valStack.push(std::sin(v));
                        else if (op == "cos") valStack.push(std::cos(v));
                        else if (op == "abs") valStack.push(std::fabs(v));
                        else if (op == "sqrt") valStack.push(std::sqrt(v));
                    }
                } else {
                    if (valStack.size() >= 2) {
                        double b = valStack.top(); valStack.pop();
                        double a = valStack.top(); valStack.pop();
                        valStack.push(applyOp(op, a, b));
                    }
                }
            }
            if (!opStack.empty() && opStack.top() == "(") opStack.pop();
            expectUnary = false;
        } else if (tok == "sin" || tok == "cos" || tok == "abs" || tok == "sqrt") {
            opStack.push(tok);
            expectUnary = true;
        } else if (isOperator(tok)) {
            if (expectUnary && (tok == "-" || tok == "+")) {
                tok = (tok == "-") ? "UNARY-" : "UNARY+";
            } else {
                while (!opStack.empty() && precedence(opStack.top()) >= precedence(tok)) {
                    std::string op = opStack.top();
                    opStack.pop();

                    if (op == "UNARY-" || op == "UNARY+") {
                        if (!valStack.empty()) {
                            double v = valStack.top(); valStack.pop();
                            valStack.push(applyOp(op, v));
                        }
                    } else if (valStack.size() >= 2) {
                        double b = valStack.top(); valStack.pop();
                        double a = valStack.top(); valStack.pop();
                        valStack.push(applyOp(op, a, b));
                    }
                }
            }
            opStack.push(tok);
            expectUnary = true;
        } else {
            // Literal or Variable
            double val = 0.0;
            auto vIt = vars.find(tok);
            if (vIt != vars.end()) {
                val = vIt->second;
            } else {
                val = parseScientific(tok);
            }
            valStack.push(val);
            expectUnary = false;
        }
    }

    while (!opStack.empty()) {
        std::string op = opStack.top();
        opStack.pop();

        if (op == "(" || op == ")") continue;

        if (op == "UNARY-" || op == "UNARY+") {
            if (!valStack.empty()) {
                double v = valStack.top(); valStack.pop();
                valStack.push(applyOp(op, v));
            }
        } else if (op == "sin" || op == "cos" || op == "abs" || op == "sqrt") {
            if (!valStack.empty()) {
                double v = valStack.top(); valStack.pop();
                if (op == "sin") valStack.push(std::sin(v));
                else if (op == "cos") valStack.push(std::cos(v));
                else if (op == "abs") valStack.push(std::fabs(v));
                else if (op == "sqrt") valStack.push(std::sqrt(v));
            }
        } else {
            if (valStack.size() >= 2) {
                double b = valStack.top(); valStack.pop();
                double a = valStack.top(); valStack.pop();
                valStack.push(applyOp(op, a, b));
            }
        }
    }

    return !valStack.empty() ? valStack.top() : 0.0;
}

} // namespace CircuitSimEngine
