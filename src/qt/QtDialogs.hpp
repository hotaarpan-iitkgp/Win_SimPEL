#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include "../engine/Components.hpp"

namespace CircuitSim {

// 1. Dynamic Configurator Dialog for SUM_RECT, SUM_ROUND, PRODUCT_RECT (AGENTS.md)
class ConfiguratorDialogQt : public QDialog {
    Q_OBJECT
public:
    explicit ConfiguratorDialogQt(ComponentInstance& comp, QWidget* parent = nullptr);

private slots:
    void onPinCountChanged(int count);
    void onApply();

private:
    ComponentInstance& m_comp;
    QSpinBox* m_spinPins;
    QVBoxLayout* m_signsLayout;
    QVector<QComboBox*> m_signBoxes;
    QCheckBox* m_checkCtrl;
};

// 2. C-Script Editor Dialog
class CScriptEditorDialogQt : public QDialog {
    Q_OBJECT
public:
    explicit CScriptEditorDialogQt(ComponentInstance& comp, QWidget* parent = nullptr);

private slots:
    void onSave();

private:
    ComponentInstance& m_comp;
    QTextEdit* m_codeEdit;
};

// 3. Unified Probe Signal Selector Dialog
class ProbeEditorDialogQt : public QDialog {
    Q_OBJECT
public:
    explicit ProbeEditorDialogQt(ComponentInstance& comp, QWidget* parent = nullptr);

private slots:
    void onSave();

private:
    ComponentInstance& m_comp;
    QLineEdit* m_targetEdit;
    QListWidget* m_signalList;
};

// 4. PWM Master Configuration Dialog
class PwmMasterDialogQt : public QDialog {
    Q_OBJECT
public:
    explicit PwmMasterDialogQt(ComponentInstance& comp, QWidget* parent = nullptr);

private slots:
    void onSave();

private:
    ComponentInstance& m_comp;
    QSpinBox* m_spinCarriers;
    QLineEdit* m_freqEdit;
    QCheckBox* m_commonModCheck;
};

} // namespace CircuitSim
