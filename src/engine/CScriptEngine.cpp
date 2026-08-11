#include "CScriptEngine.hpp"
#include "ExpressionEvaluator.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <regex>
#include <unordered_set>

namespace CircuitSimEngine {

static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n{}");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n{}");
    return str.substr(first, (last - first + 1));
}

static std::string stripComments(const std::string& code) {
    std::string result;
    bool inLineComment = false;
    bool inBlockComment = false;

    for (size_t i = 0; i < code.size(); ++i) {
        if (inLineComment) {
            if (code[i] == '\n') {
                inLineComment = false;
                result += '\n';
            }
        } else if (inBlockComment) {
            if (code[i] == '*' && i + 1 < code.size() && code[i + 1] == '/') {
                inBlockComment = false;
                i++;
            }
        } else {
            if (code[i] == '/' && i + 1 < code.size() && code[i + 1] == '/') {
                inLineComment = true;
                i++;
            } else if (code[i] == '/' && i + 1 < code.size() && code[i + 1] == '*') {
                inBlockComment = true;
                i++;
            } else {
                result += code[i];
            }
        }
    }
    return result;
}

static int precedence(const std::string& op) {
    if (op == "||") return 1;
    if (op == "&&") return 2;
    if (op == "==" || op == "!=") return 3;
    if (op == "<" || op == ">" || op == "<=" || op == ">=") return 4;
    if (op == "+" || op == "-") return 5;
    if (op == "*" || op == "/") return 6;
    if (op == "UNARY-") return 7;
    return 0;
}

int CScriptEngine::getOrRegisterVar(const std::string& name, double initVal) {
    auto it = varNameToIdx.find(name);
    if (it != varNameToIdx.end()) {
        return it->second;
    }
    int idx = (int)flatVars.size();
    flatVars.push_back(initVal);
    varNameToIdx[name] = idx;
    idxToVarName.push_back(name);
    return idx;
}

FastCompiledRPNExpr CScriptEngine::compileToRPN(const std::string& exprStr) {
    FastCompiledRPNExpr rpn;
    std::string s = trim(exprStr);
    if (s.empty()) return rpn;

    // Check for ternary
    size_t qPos = s.find('?');
    if (qPos != std::string::npos) {
        int depth = 0;
        size_t cPos = std::string::npos;
        for (size_t i = qPos + 1; i < s.size(); ++i) {
            if (s[i] == '(') depth++;
            else if (s[i] == ')') depth--;
            else if (s[i] == ':' && depth == 0) { cPos = i; break; }
        }
        if (cPos != std::string::npos) {
            std::string condStr = s.substr(0, qPos);
            std::string thenStr = s.substr(qPos + 1, cPos - qPos - 1);
            std::string elseStr = s.substr(cPos + 1);

            auto condRPN = compileToRPN(condStr);
            auto thenRPN = compileToRPN(thenStr);
            auto elseRPN = compileToRPN(elseStr);

            rpn.fastTokens.insert(rpn.fastTokens.end(), condRPN.fastTokens.begin(), condRPN.fastTokens.end());
            rpn.fastTokens.insert(rpn.fastTokens.end(), thenRPN.fastTokens.begin(), thenRPN.fastTokens.end());
            rpn.fastTokens.insert(rpn.fastTokens.end(), elseRPN.fastTokens.begin(), elseRPN.fastTokens.end());

            FastOpToken ternaryTok;
            ternaryTok.op = OpCode::Ternary;
            rpn.fastTokens.push_back(ternaryTok);
            return rpn;
        }
    }

    std::vector<std::string> infixTokens;
    size_t i = 0;
    while (i < s.size()) {
        if (std::isspace(s[i])) { i++; continue; }

        if (s[i] == '+' || s[i] == '-' || s[i] == '*' || s[i] == '/' || s[i] == '%' || s[i] == '(' || s[i] == ')' || s[i] == ',') {
            if (i + 1 < s.size()) {
                std::string sub2 = s.substr(i, 2);
                if (sub2 == "==" || sub2 == "!=" || sub2 == "<=" || sub2 == ">=" || sub2 == "&&" || sub2 == "||") {
                    infixTokens.push_back(sub2);
                    i += 2;
                    continue;
                }
            }
            infixTokens.push_back(std::string(1, s[i]));
            i++;
        } else if (s[i] == '<' || s[i] == '>' || s[i] == '!') {
            if (i + 1 < s.size() && s[i + 1] == '=') {
                infixTokens.push_back(s.substr(i, 2));
                i += 2;
            } else {
                infixTokens.push_back(std::string(1, s[i]));
                i++;
            }
        } else if (s[i] == '&' && i + 1 < s.size() && s[i + 1] == '&') {
            infixTokens.push_back("&&");
            i += 2;
        } else if (s[i] == '|' && i + 1 < s.size() && s[i + 1] == '|') {
            infixTokens.push_back("||");
            i += 2;
        } else {
            size_t start = i;
            int bracketDepth = 0;
            while (i < s.size()) {
                if (s[i] == '[') bracketDepth++;
                else if (s[i] == ']') bracketDepth--;
                else if (bracketDepth == 0 && (std::isspace(s[i]) || s[i] == '+' || s[i] == '-' || s[i] == '*' || s[i] == '/' || s[i] == '%' || s[i] == '(' || s[i] == ')' || s[i] == ',' || s[i] == '<' || s[i] == '>' || s[i] == '=' || s[i] == '!' || s[i] == '&' || s[i] == '|' || s[i] == '?')) {
                    break;
                }
                i++;
            }
            infixTokens.push_back(s.substr(start, i - start));
        }
    }

    std::vector<std::string> opStack;
    bool expectUnary = true;

    auto pushOpToken = [&](const std::string& opStr) {
        FastOpToken tok;
        if (opStr == "+") tok.op = OpCode::Add;
        else if (opStr == "-") tok.op = OpCode::Sub;
        else if (opStr == "*") tok.op = OpCode::Mul;
        else if (opStr == "/") tok.op = OpCode::Div;
        else if (opStr == "%") tok.op = OpCode::Mod;
        else if (opStr == "UNARY-") tok.op = OpCode::UnaryMinus;
        else if (opStr == "sin") tok.op = OpCode::Sin;
        else if (opStr == "cos") tok.op = OpCode::Cos;
        else if (opStr == "tan") tok.op = OpCode::Tan;
        else if (opStr == "abs") tok.op = OpCode::Abs;
        else if (opStr == "sqrt") tok.op = OpCode::Sqrt;
        else if (opStr == "exp") tok.op = OpCode::Exp;
        else if (opStr == "log") tok.op = OpCode::Log;
        else if (opStr == "max") tok.op = OpCode::Max;
        else if (opStr == "min") tok.op = OpCode::Min;
        else if (opStr == "pow") tok.op = OpCode::Pow;
        else if (opStr == "<") tok.op = OpCode::Lt;
        else if (opStr == ">") tok.op = OpCode::Gt;
        else if (opStr == "<=") tok.op = OpCode::LtEq;
        else if (opStr == ">=") tok.op = OpCode::GtEq;
        else if (opStr == "==") tok.op = OpCode::Eq;
        else if (opStr == "!=") tok.op = OpCode::NotEq;
        else if (opStr == "&&") tok.op = OpCode::And;
        else if (opStr == "||") tok.op = OpCode::Or;
        rpn.fastTokens.push_back(tok);
    };

    for (const auto& tok : infixTokens) {
        if (tok == "(") {
            opStack.push_back(tok);
            expectUnary = true;
        } else if (tok == ",") {
            while (!opStack.empty() && opStack.back() != "(") {
                pushOpToken(opStack.back());
                opStack.pop_back();
            }
            expectUnary = true;
        } else if (tok == ")") {
            while (!opStack.empty() && opStack.back() != "(") {
                pushOpToken(opStack.back());
                opStack.pop_back();
            }
            if (!opStack.empty() && opStack.back() == "(") opStack.pop_back();
            if (!opStack.empty() && (opStack.back() == "sin" || opStack.back() == "cos" || opStack.back() == "tan" ||
                                     opStack.back() == "abs" || opStack.back() == "fabs" || opStack.back() == "sqrt" ||
                                     opStack.back() == "max" || opStack.back() == "min" || opStack.back() == "pow" ||
                                     opStack.back() == "exp" || opStack.back() == "log" || opStack.back() == "asin" ||
                                     opStack.back() == "acos" || opStack.back() == "atan")) {
                pushOpToken(opStack.back());
                opStack.pop_back();
            }
            expectUnary = false;
        } else if (tok == "sin" || tok == "cos" || tok == "tan" || tok == "abs" || tok == "sqrt" || tok == "max" || tok == "min" || tok == "pow" || tok == "exp" || tok == "log") {
            opStack.push_back(tok);
            expectUnary = true;
        } else if (tok == "+" || tok == "-" || tok == "*" || tok == "/" || tok == "%" ||
                   tok == "<" || tok == ">" || tok == "<=" || tok == ">=" || tok == "==" || tok == "!=" ||
                   tok == "&&" || tok == "||") {
            std::string op = tok;
            if (expectUnary && (op == "-" || op == "+")) {
                op = (op == "-") ? "UNARY-" : "UNARY+";
            } else {
                while (!opStack.empty() && precedence(opStack.back()) >= precedence(op)) {
                    pushOpToken(opStack.back());
                    opStack.pop_back();
                }
            }
            opStack.push_back(op);
            expectUnary = true;
        } else {
            FastOpToken rTok;
            bool isNum = true;
            try {
                rTok.numVal = ExpressionEvaluator::parseScientific(tok);
                rTok.op = OpCode::PushNum;
            } catch (...) {
                isNum = false;
            }

            if (isNum && (std::isdigit(tok[0]) || tok[0] == '.')) {
                rTok.op = OpCode::PushNum;
            } else {
                if (tok.substr(0, 7) == "inputs[") {
                    size_t bracket = tok.find(']');
                    if (bracket != std::string::npos) {
                        rTok.op = OpCode::PushInput;
                        rTok.inOutIdx = std::stoi(tok.substr(7, bracket - 7));
                    }
                } else if (tok.substr(0, 8) == "outputs[") {
                    size_t bracket = tok.find(']');
                    if (bracket != std::string::npos) {
                        rTok.op = OpCode::PushOutput;
                        rTok.inOutIdx = std::stoi(tok.substr(8, bracket - 8));
                    }
                } else if (tok == "PI" || tok == "pi" || tok == "M_PI") {
                    rTok.numVal = 3.14159265358979323846;
                    rTok.op = OpCode::PushNum;
                } else {
                    rTok.op = OpCode::PushVar;
                    rTok.varIdx = getOrRegisterVar(tok, 0.0);
                }
            }

            rpn.fastTokens.push_back(rTok);
            expectUnary = false;
        }
    }

    while (!opStack.empty()) {
        std::string op = opStack.back();
        opStack.pop_back();
        if (op != "(" && op != ")") {
            pushOpToken(op);
        }
    }

    return rpn;
}

double CScriptEngine::evalRPN(const FastCompiledRPNExpr& rpn) {
    double stack[64];
    int top = 0;

    for (const auto& tok : rpn.fastTokens) {
        switch (tok.op) {
            case OpCode::PushNum: stack[top++] = tok.numVal; break;
            case OpCode::PushVar: stack[top++] = (tok.varIdx >= 0 && tok.varIdx < (int)flatVars.size()) ? flatVars[tok.varIdx] : 0.0; break;
            case OpCode::PushInput: stack[top++] = (tok.inOutIdx >= 0 && tok.inOutIdx < (int)inputs.size()) ? inputs[tok.inOutIdx] : 0.0; break;
            case OpCode::PushOutput: stack[top++] = (tok.inOutIdx >= 0 && tok.inOutIdx < (int)outputs.size()) ? outputs[tok.inOutIdx] : 0.0; break;
            case OpCode::Add: top--; stack[top - 1] += stack[top]; break;
            case OpCode::Sub: top--; stack[top - 1] -= stack[top]; break;
            case OpCode::Mul: top--; stack[top - 1] *= stack[top]; break;
            case OpCode::Div: top--; stack[top - 1] = (std::abs(stack[top]) > 1e-15) ? (stack[top - 1] / stack[top]) : 0.0; break;
            case OpCode::Mod: top--; stack[top - 1] = (stack[top] != 0) ? std::fmod(stack[top - 1], stack[top]) : 0.0; break;
            case OpCode::UnaryMinus: stack[top - 1] = -stack[top - 1]; break;
            case OpCode::Sin: stack[top - 1] = std::sin(stack[top - 1]); break;
            case OpCode::Cos: stack[top - 1] = std::cos(stack[top - 1]); break;
            case OpCode::Tan: stack[top - 1] = std::tan(stack[top - 1]); break;
            case OpCode::Abs: stack[top - 1] = std::fabs(stack[top - 1]); break;
            case OpCode::Sqrt: stack[top - 1] = std::sqrt(std::fabs(stack[top - 1])); break;
            case OpCode::Exp: stack[top - 1] = std::exp(stack[top - 1]); break;
            case OpCode::Log: stack[top - 1] = std::log(std::fabs(stack[top - 1]) + 1e-15); break;
            case OpCode::Max: top--; stack[top - 1] = std::max(stack[top - 1], stack[top]); break;
            case OpCode::Min: top--; stack[top - 1] = std::min(stack[top - 1], stack[top]); break;
            case OpCode::Pow: top--; stack[top - 1] = std::pow(stack[top - 1], stack[top]); break;
            case OpCode::Lt: top--; stack[top - 1] = (stack[top - 1] < stack[top]) ? 1.0 : 0.0; break;
            case OpCode::Gt: top--; stack[top - 1] = (stack[top - 1] > stack[top]) ? 1.0 : 0.0; break;
            case OpCode::LtEq: top--; stack[top - 1] = (stack[top - 1] <= stack[top]) ? 1.0 : 0.0; break;
            case OpCode::GtEq: top--; stack[top - 1] = (stack[top - 1] >= stack[top]) ? 1.0 : 0.0; break;
            case OpCode::Eq: top--; stack[top - 1] = (std::abs(stack[top - 1] - stack[top]) < 1e-9) ? 1.0 : 0.0; break;
            case OpCode::NotEq: top--; stack[top - 1] = (std::abs(stack[top - 1] - stack[top]) >= 1e-9) ? 1.0 : 0.0; break;
            case OpCode::And: top--; stack[top - 1] = (stack[top - 1] != 0.0 && stack[top] != 0.0) ? 1.0 : 0.0; break;
            case OpCode::Or: top--; stack[top - 1] = (stack[top - 1] != 0.0 || stack[top] != 0.0) ? 1.0 : 0.0; break;
            case OpCode::Ternary: {
                if (top >= 3) {
                    double elseVal = stack[--top];
                    double thenVal = stack[--top];
                    double condVal = stack[--top];
                    stack[top++] = (std::abs(condVal) > 1e-12) ? thenVal : elseVal;
                }
                break;
            }
        }
    }

    return (top > 0) ? stack[top - 1] : 0.0;
}

static std::pair<std::string, std::string> parseSingleAssignmentStrings(const std::string& stmtStr) {
    std::string clean = trim(stmtStr);
    if (clean.empty()) return {"", ""};
    if (clean.back() == ';') clean.pop_back();
    clean = trim(clean);

    size_t eqPos = clean.find('=');
    if (eqPos == std::string::npos) return {"", ""};

    std::string lhs = trim(clean.substr(0, eqPos));
    std::string rhs = trim(clean.substr(eqPos + 1));

    if (!lhs.empty() && (lhs.back() == '+' || lhs.back() == '-' || lhs.back() == '*' || lhs.back() == '/')) {
        char op = lhs.back();
        lhs = trim(lhs.substr(0, lhs.size() - 1));
        rhs = lhs + " " + op + " (" + rhs + ")";
    }

    const std::vector<std::string> prefixes = {"const double", "double", "float", "int", "auto", "const float", "const int"};
    for (const auto& pref : prefixes) {
        if (lhs.substr(0, pref.size()) == pref && (lhs.size() == pref.size() || std::isspace(lhs[pref.size()]))) {
            lhs = trim(lhs.substr(pref.size()));
            break;
        }
    }

    return {lhs, rhs};
}

static std::string extractNextStatementBody(const std::string& code, size_t startPos, size_t& nextPos) {
    size_t i = startPos;
    size_t n = code.size();
    while (i < n && std::isspace(code[i])) i++;
    if (i >= n) { nextPos = n; return ""; }

    if (code[i] == '{') {
        int depth = 1;
        size_t bStart = i;
        size_t bEnd = std::string::npos;
        for (size_t j = bStart + 1; j < n; ++j) {
            if (code[j] == '{') depth++;
            else if (code[j] == '}') {
                depth--;
                if (depth == 0) { bEnd = j; break; }
            }
        }
        if (bEnd != std::string::npos) {
            nextPos = bEnd + 1;
            return code.substr(bStart + 1, bEnd - bStart - 1);
        }
    }

    if (i + 2 <= n && code.substr(i, 2) == "if" && (i + 2 == n || (!std::isalnum(code[i+2]) && code[i+2] != '_'))) {
        size_t pStart = code.find('(', i);
        if (pStart != std::string::npos) {
            int depth = 1;
            size_t pEnd = std::string::npos;
            for (size_t j = pStart + 1; j < n; ++j) {
                if (code[j] == '(') depth++;
                else if (code[j] == ')') { depth--; if (depth == 0) { pEnd = j; break; } }
            }
            if (pEnd != std::string::npos) {
                size_t afterCond = pEnd + 1;
                size_t endThen = 0;
                extractNextStatementBody(code, afterCond, endThen);
                size_t checkElse = endThen;
                while (checkElse < n && std::isspace(code[checkElse])) checkElse++;
                if (checkElse + 4 <= n && code.substr(checkElse, 4) == "else" && (checkElse + 4 == n || (!std::isalnum(code[checkElse+4]) && code[checkElse+4] != '_'))) {
                    size_t endElse = 0;
                    extractNextStatementBody(code, checkElse + 4, endElse);
                    nextPos = endElse;
                    return code.substr(i, endElse - i);
                }
                nextPos = endThen;
                return code.substr(i, endThen - i);
            }
        }
    }

    size_t sEnd = code.find(';', i);
    if (sEnd != std::string::npos) {
        nextPos = sEnd + 1;
        return code.substr(i, sEnd - i + 1);
    }

    nextPos = n;
    return code.substr(i);
}

std::vector<FastCompiledCScriptStmt> CScriptEngine::parseBlockStatements(const std::string& code) {
    std::vector<FastCompiledCScriptStmt> stmts;
    size_t i = 0;
    size_t n = code.size();

    while (i < n) {
        while (i < n && std::isspace(code[i])) i++;
        if (i >= n) break;

        // Check for 'if' statement
        if (i + 2 <= n && code.substr(i, 2) == "if" && (i + 2 == n || (!std::isalnum(code[i+2]) && code[i+2] != '_'))) {
            size_t pStart = code.find('(', i);
            if (pStart != std::string::npos) {
                int depth = 1;
                size_t pEnd = std::string::npos;
                for (size_t j = pStart + 1; j < n; ++j) {
                    if (code[j] == '(') depth++;
                    else if (code[j] == ')') {
                        depth--;
                        if (depth == 0) { pEnd = j; break; }
                    }
                }

                if (pEnd != std::string::npos) {
                    std::string condStr = code.substr(pStart + 1, pEnd - pStart - 1);
                    FastCompiledCScriptStmt ifStmt;
                    ifStmt.type = FastCompiledCScriptStmt::Type::IfBlock;
                    ifStmt.condRPN = compileToRPN(condStr);

                    size_t nextI = 0;
                    std::string thenStr = extractNextStatementBody(code, pEnd + 1, nextI);
                    i = nextI;

                    ifStmt.thenBody = parseBlockStatements(thenStr);

                    while (i < n && std::isspace(code[i])) i++;

                    // Check for optional 'else'
                    if (i + 4 <= n && code.substr(i, 4) == "else" && (i + 4 == n || (!std::isalnum(code[i+4]) && code[i+4] != '_'))) {
                        size_t endElse = 0;
                        std::string elseStr = extractNextStatementBody(code, i + 4, endElse);
                        i = endElse;

                        ifStmt.elseBody = parseBlockStatements(elseStr);
                    }

                    stmts.push_back(ifStmt);
                    continue;
                }
            }
        }

        // Single assignment statement up to ';'
        size_t semi = code.find(';', i);
        if (semi != std::string::npos) {
            std::string stmtStr = code.substr(i, semi - i);
            i = semi + 1;
            auto pair = parseSingleAssignmentStrings(stmtStr);
            if (!pair.first.empty()) {
                FastCompiledCScriptStmt assignStmt;
                assignStmt.type = FastCompiledCScriptStmt::Type::Assignment;
                assignStmt.lhsName = pair.first;
                if (pair.first.substr(0, 8) == "outputs[") {
                    assignStmt.isLhsOutput = true;
                    size_t bracket = pair.first.find(']');
                    if (bracket != std::string::npos) assignStmt.lhsOutputIdx = std::stoi(pair.first.substr(8, bracket - 8));
                } else {
                    assignStmt.lhsVarIdx = getOrRegisterVar(pair.first, 0.0);
                }
                assignStmt.rhsRPN = compileToRPN(pair.second);
                stmts.push_back(assignStmt);
            }
        } else {
            break;
        }
    }

    return stmts;
}

void CScriptEngine::setup(const std::string& inputCode, const std::unordered_map<std::string, std::string>& overrideParams) {
    codeStr = inputCode;
    timestep = 0.0;
    if (overrideParams.count("timestep")) {
        try { timestep = ExpressionEvaluator::parseScientific(overrideParams.at("timestep")); } catch (...) {}
    }
    nextSampleTime = 0.0;

    std::string cleanCode = stripComments(inputCode);
    flatVars.clear();
    varNameToIdx.clear();
    idxToVarName.clear();
    compiledStmts.clear();
    outputs.assign(20, 0.0);
    inputs.assign(20, 0.0);

    getOrRegisterVar("PI", 3.14159265358979323846);
    getOrRegisterVar("pi", 3.14159265358979323846);
    getOrRegisterVar("M_PI", 3.14159265358979323846);

    for (const auto& [k, v] : overrideParams) {
        if (!v.empty() && k != "code" && k != "id" && k != "type" && k != "inputs" && k != "outputs" && k != "timestep") {
            getOrRegisterVar(k, ExpressionEvaluator::parseScientific(v));
        }
    }

    size_t stepPos = cleanCode.find("void step()");
    if (stepPos == std::string::npos) stepPos = cleanCode.find("step()");

    std::string bodyCode = cleanCode;
    if (stepPos != std::string::npos) {
        size_t braceStart = cleanCode.find('{', stepPos);
        if (braceStart != std::string::npos) {
            size_t braceEnd = cleanCode.rfind('}');
            if (braceEnd != std::string::npos && braceEnd > braceStart) {
                std::string headerCode = cleanCode.substr(0, stepPos);
                std::stringstream hss(headerCode);
                std::string hline;
                while (std::getline(hss, hline, ';')) {
                    hline = trim(hline);
                    if (!hline.empty()) {
                        auto pair = parseSingleAssignmentStrings(hline);
                        if (!pair.first.empty()) {
                            if (varNameToIdx.find(pair.first) == varNameToIdx.end()) {
                                auto rpn = compileToRPN(pair.second);
                                double val = evalRPN(rpn);
                                getOrRegisterVar(pair.first, val);
                            }
                        }
                    }
                }

                bodyCode = cleanCode.substr(braceStart + 1, braceEnd - braceStart - 1);
            }
        }
    }

    compiledStmts = parseBlockStatements(bodyCode);
}

void CScriptEngine::execCompiledStmts(const std::vector<FastCompiledCScriptStmt>& stmts) {
    for (const auto& stmt : stmts) {
        if (stmt.type == FastCompiledCScriptStmt::Type::Assignment) {
            double val = evalRPN(stmt.rhsRPN);
            if (stmt.isLhsOutput) {
                if (stmt.lhsOutputIdx >= 0 && stmt.lhsOutputIdx < (int)outputs.size()) outputs[stmt.lhsOutputIdx] = val;
            } else if (stmt.lhsVarIdx >= 0 && stmt.lhsVarIdx < (int)flatVars.size()) {
                flatVars[stmt.lhsVarIdx] = val;
            }
        } else if (stmt.type == FastCompiledCScriptStmt::Type::IfBlock) {
            double condVal = evalRPN(stmt.condRPN);
            if (std::abs(condVal) > 1e-12) {
                execCompiledStmts(stmt.thenBody);
            } else {
                execCompiledStmts(stmt.elseBody);
            }
        }
    }
}

void CScriptEngine::step(double currentTime, const std::vector<double>& inVals, double dt) {
    if (timestep > 1e-12) {
        if (currentTime < nextSampleTime - 1e-12 && currentTime > 1e-12) {
            return;
        }
        nextSampleTime = currentTime + timestep;
    }

    inputs = inVals;
    if (inputs.size() < 20) inputs.resize(20, 0.0);

    int timeIdx = getOrRegisterVar("time", currentTime);
    int dtIdx = getOrRegisterVar("dt", dt);
    flatVars[timeIdx] = currentTime;
    flatVars[dtIdx] = dt;

    execCompiledStmts(compiledStmts);
}

double CScriptEngine::getOutput(size_t index) const {
    if (index < outputs.size()) return outputs[index];
    return 0.0;
}

double CScriptEngine::getOutputByName(const std::string& name) const {
    auto it = namedOutputToIdx.find(name);
    if (it != namedOutputToIdx.end() && it->second < (int)outputs.size()) {
        return outputs[it->second];
    }
    return getVar(name);
}

double CScriptEngine::getVar(const std::string& name) const {
    auto it = varNameToIdx.find(name);
    if (it != varNameToIdx.end() && it->second < (int)flatVars.size()) return flatVars[it->second];
    return 0.0;
}

std::unordered_map<std::string, double> CScriptEngine::getAllVars() const {
    std::unordered_map<std::string, double> res;
    for (size_t i = 0; i < flatVars.size(); ++i) {
        if (i < idxToVarName.size()) res[idxToVarName[i]] = flatVars[i];
    }
    return res;
}

void CScriptEngine::discoverPorts(const std::string& code, std::vector<CScriptPort>& outInputs, std::vector<CScriptPort>& outOutputs) {
    outInputs.clear();
    outOutputs.clear();

    std::unordered_set<std::string> seenIn;
    std::unordered_set<std::string> seenOut;

    std::regex inNamedRegex(R"(inputs\s*\[\s*["']([^"']+)["']\s*\]|inputs\.get\s*\(\s*["']([^"']+)["']\s*\))");
    std::regex inIdxRegex(R"(inputs\s*\[\s*(\d+)\s*\])");

    std::regex outNamedRegex(R"(outputs\s*\[\s*["']([^"']+)["']\s*\]|outputs\.set\s*\(\s*["']([^"']+)["']\s*,\s*|outputs\s*\.\s*([a-zA-Z0-9_]+))");
    std::regex outIdxRegex(R"(outputs\s*\[\s*(\d+)\s*\])");

    auto inBegin = std::sregex_iterator(code.begin(), code.end(), inNamedRegex);
    auto inEnd = std::sregex_iterator();
    for (std::sregex_iterator i = inBegin; i != inEnd; ++i) {
        std::smatch m = *i;
        std::string pName = m[1].matched ? m[1].str() : m[2].str();
        if (!pName.empty() && !seenIn.count(pName)) {
            seenIn.insert(pName);
            outInputs.push_back({pName, false, (int)outInputs.size()});
        }
    }

    auto inIdxBegin = std::sregex_iterator(code.begin(), code.end(), inIdxRegex);
    for (std::sregex_iterator i = inIdxBegin; i != inEnd; ++i) {
        std::smatch m = *i;
        int idx = std::stoi(m[1].str());
        std::string pName = (idx == 0 && outInputs.empty()) ? "In1" : ("In" + std::to_string(idx + 1));
        if (!seenIn.count(pName)) {
            seenIn.insert(pName);
            outInputs.push_back({pName, false, idx});
        }
    }

    auto outBegin = std::sregex_iterator(code.begin(), code.end(), outNamedRegex);
    for (std::sregex_iterator i = outBegin; i != inEnd; ++i) {
        std::smatch m = *i;
        std::string pName = m[1].matched ? m[1].str() : (m[2].matched ? m[2].str() : m[3].str());
        if (!pName.empty() && pName != "set" && !seenOut.count(pName)) {
            seenOut.insert(pName);
            outOutputs.push_back({pName, true, (int)outOutputs.size()});
        }
    }

    auto outIdxBegin = std::sregex_iterator(code.begin(), code.end(), outIdxRegex);
    for (std::sregex_iterator i = outIdxBegin; i != inEnd; ++i) {
        std::smatch m = *i;
        int idx = std::stoi(m[1].str());
        std::string pName = (idx == 0 && outOutputs.empty()) ? "Out1" : ("Out" + std::to_string(idx + 1));
        if (!seenOut.count(pName)) {
            seenOut.insert(pName);
            outOutputs.push_back({pName, true, idx});
        }
    }

    if (outInputs.empty()) {
        outInputs.push_back({"In1", false, 0});
    }
    if (outOutputs.empty()) {
        outOutputs.push_back({"Out1", true, 0});
    }
}

std::vector<CScriptParam> CScriptEngine::discoverParamsFromCode(const std::string& code) {
    std::vector<CScriptParam> res;
    std::stringstream ss(code);
    std::string line;
    std::regex paramRegex(R"(^\s*(const\s+)?(double|float|int)\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*=\s*([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)\s*;)");

    while (std::getline(ss, line)) {
        std::smatch m;
        if (std::regex_search(line, m, paramRegex)) {
            CScriptParam p;
            p.typeStr = (m[1].matched ? m[1].str() : "") + m[2].str();
            p.name = m[3].str();
            p.rawValStr = m[4].str();
            try { p.value = ExpressionEvaluator::parseScientific(p.rawValStr); } catch (...) { p.value = 0.0; }
            res.push_back(p);
        }
    }
    return res;
}

std::string CScriptEngine::updateParamInCode(const std::string& code, const std::string& paramName, double newValue) {
    std::stringstream ss(code);
    std::stringstream outCode;
    std::string line;
    std::regex paramRegex(R"(^(\s*(?:const\s+)?(?:double|float|int)\s+)" + paramName + R"(\s*=\s*)([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)(.*)$)");

    char valStrBuf[64];
    snprintf(valStrBuf, sizeof(valStrBuf), "%.9g", newValue);

    while (std::getline(ss, line)) {
        std::smatch m;
        if (std::regex_match(line, m, paramRegex)) {
            outCode << m[1].str() << valStrBuf << m[3].str() << "\n";
        } else {
            outCode << line << "\n";
        }
    }
    return outCode.str();
}

} // namespace CircuitSimEngine
