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

struct CScriptPort {
    std::string name;
    bool isOutput = false;
    int index = 0;
};

struct CScriptParam {
    std::string name;
    double value = 0.0;
    std::string typeStr = "double";
    std::string rawValStr;
};

class CScriptEngine {
private:
    std::string codeStr;
    double timestep = 0.0;
    double nextSampleTime = 0.0;
    std::vector<double> flatVars;
    std::unordered_map<std::string, int> varNameToIdx;
    std::vector<std::string> idxToVarName;

    std::vector<double> inputs;
    std::vector<double> outputs;
    std::unordered_map<std::string, int> namedInputToIdx;
    std::unordered_map<std::string, int> namedOutputToIdx;
    std::vector<FastCompiledCScriptStmt> compiledStmts;

    int getOrRegisterVar(const std::string& name, double initVal = 0.0);
    double evalRPN(const FastCompiledRPNExpr& rpn);
    FastCompiledRPNExpr compileToRPN(const std::string& exprStr);
    std::vector<FastCompiledCScriptStmt> parseBlockStatements(const std::string& code);
    void execCompiledStmts(const std::vector<FastCompiledCScriptStmt>& stmts);

public:
    CScriptEngine() = default;

    static void discoverPorts(const std::string& code, std::vector<CScriptPort>& outInputs, std::vector<CScriptPort>& outOutputs);
    static std::vector<CScriptParam> discoverParamsFromCode(const std::string& code);
    static std::string updateParamInCode(const std::string& code, const std::string& paramName, double newValue);

    void setup(const std::string& inputCode, const std::unordered_map<std::string, std::string>& overrideParams);
    void step(double currentTime, const std::vector<double>& inVals, double dt);

    double getOutput(size_t index) const;
    double getOutputByName(const std::string& name) const;
    double getVar(const std::string& name) const;
    std::unordered_map<std::string, double> getAllVars() const;
};

} // namespace CircuitSimEngine
