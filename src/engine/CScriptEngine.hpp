#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cstdint>

namespace CircuitSimEngine {

enum class OpCode : uint8_t {
    PushNum,
    PushVar,
    PushInput,
    PushOutput,
    Add, Sub, Mul, Div, Mod,
    UnaryMinus,
    Sin, Cos, Tan, Abs, Sqrt, Exp, Log,
    Max, Min, Pow,
    Lt, Gt, LtEq, GtEq, Eq, NotEq,
    And, Or,
    Ternary
};

struct FastOpToken {
    OpCode op = OpCode::PushNum;
    double numVal = 0.0;
    int varIdx = -1;
    int inOutIdx = 0;
};

struct FastCompiledRPNExpr {
    std::vector<FastOpToken> fastTokens;
};

struct FastCompiledCScriptStmt {
    enum class Type { Assignment, IfBlock } type = Type::Assignment;
    int lhsVarIdx = -1;
    bool isLhsOutput = false;
    int lhsOutputIdx = 0;
    std::string lhsName;

    FastCompiledRPNExpr rhsRPN;
    
    // For IfBlock
    FastCompiledRPNExpr condRPN;
    std::vector<FastCompiledCScriptStmt> thenBody;
    std::vector<FastCompiledCScriptStmt> elseBody;
};

class CScriptEngine {
private:
    std::string codeStr;
    std::vector<double> flatVars;
    std::unordered_map<std::string, int> varNameToIdx;
    std::vector<std::string> idxToVarName;

    std::vector<double> inputs;
    std::vector<double> outputs;
    std::vector<FastCompiledCScriptStmt> compiledStmts;

    int getOrRegisterVar(const std::string& name, double initVal = 0.0);
    double evalRPN(const FastCompiledRPNExpr& rpn);
    FastCompiledRPNExpr compileToRPN(const std::string& exprStr);
    std::vector<FastCompiledCScriptStmt> parseBlockStatements(const std::string& code);
    void execCompiledStmts(const std::vector<FastCompiledCScriptStmt>& stmts);

public:
    CScriptEngine() = default;

    void setup(const std::string& inputCode, const std::unordered_map<std::string, std::string>& overrideParams);
    void step(double currentTime, const std::vector<double>& inVals, double dt);

    double getOutput(size_t index) const;
    double getVar(const std::string& name) const;
    std::unordered_map<std::string, double> getAllVars() const;
};

} // namespace CircuitSimEngine
