#include "QtDialogs.hpp"
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>

namespace CircuitSim {

// ─── 1. Dynamic Configurator Dialog (AGENTS.md) ───
ConfiguratorDialogQt::ConfiguratorDialogQt(ComponentInstance& comp, QWidget* parent)
    : QDialog(parent), m_comp(comp)
{
    setWindowTitle(QString("Configure %1 Pin Topology").arg(QString::fromStdString(comp.rawTypeStr)));
    resize(380, 320);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QFormLayout* form = new QFormLayout();
    m_spinPins = new QSpinBox(this);
    m_spinPins->setRange(1, 20);
    m_spinPins->setValue(comp.numInputPins > 0 ? comp.numInputPins : 2);
    form->addRow("Input Pins Count:", m_spinPins);

    if (comp.rawTypeStr == "SUM_RECT" || comp.rawTypeStr == "PRODUCT_RECT") {
        m_checkCtrl = new QCheckBox("Enable Top-Left CTRL Pin", this);
        m_checkCtrl->setChecked(comp.hasCtrlPin);
        form->addRow("Control Pin:", m_checkCtrl);
    } else {
        m_checkCtrl = nullptr;
    }

    mainLayout->addLayout(form);

    QGroupBox* signsGroup = new QGroupBox("Pin Operator Signs", this);
    m_signsLayout = new QVBoxLayout(signsGroup);
    signsGroup->setLayout(m_signsLayout);
    mainLayout->addWidget(signsGroup);

    connect(m_spinPins, QOverload<int>::of(&QSpinBox::valueChanged), this, &ConfiguratorDialogQt::onPinCountChanged);
    onPinCountChanged(m_spinPins->value());

    QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(bbox, &QDialogButtonBox::accepted, this, &ConfiguratorDialogQt::onApply);
    connect(bbox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(bbox);
}

void ConfiguratorDialogQt::onPinCountChanged(int count) {
    QLayoutItem* child;
    while ((child = m_signsLayout->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }
    m_signBoxes.clear();

    for (int i = 1; i <= count; ++i) {
        QHBoxLayout* row = new QHBoxLayout();
        row->addWidget(new QLabel(QString("Input Pin %1 Sign:").arg(i)));

        QComboBox* cbox = new QComboBox(this);
        if (m_comp.rawTypeStr.find("PRODUCT") != std::string::npos) {
            cbox->addItem("× (Multiply)");
            cbox->addItem("/ (Divide)");
        } else {
            cbox->addItem("+ (Add)");
            cbox->addItem("- (Subtract)");
        }

        if (i - 1 < m_comp.pinSigns.size()) {
            if (m_comp.pinSigns[i - 1] == "-" || m_comp.pinSigns[i - 1] == "/") {
                cbox->setCurrentIndex(1);
            }
        }

        row->addWidget(cbox);
        m_signsLayout->addLayout(row);
        m_signBoxes.append(cbox);
    }
}

void ConfiguratorDialogQt::onApply() {
    int count = m_spinPins->value();
    m_comp.numInputPins = count;
    m_comp.pinSigns.clear();
    m_comp.pins.clear();

    float height = std::max(40.0f, count * 20.0f);
    m_comp.height = height;

    for (int i = 0; i < count; ++i) {
        QString sign = (m_signBoxes[i]->currentIndex() == 0) ? 
            (m_comp.rawTypeStr.find("PRODUCT") != std::string::npos ? "×" : "+") :
            (m_comp.rawTypeStr.find("PRODUCT") != std::string::npos ? "/" : "-");

        m_comp.pinSigns.push_back(sign.toStdString());

        Pin pin;
        pin.name = "In" + std::to_string(i + 1);
        pin.relativeX = -25.0f;
        pin.relativeY = - (count - 1) * 10.0f + i * 20.0f;
        pin.isInput = true;
        pin.opSign = sign.toStdString();
        m_comp.pins.push_back(pin);
    }

    Pin outPin;
    outPin.name = "Out";
    outPin.relativeX = 25.0f;
    outPin.relativeY = 0.0f;
    outPin.isOutput = true;
    m_comp.pins.push_back(outPin);

    if (m_checkCtrl && m_checkCtrl->isChecked()) {
        m_comp.hasCtrlPin = true;
        Pin ctrlPin;
        ctrlPin.name = "Ctrl";
        ctrlPin.relativeX = -25.0f;
        ctrlPin.relativeY = -height * 0.5f;
        ctrlPin.isCtrl = true;
        m_comp.pins.push_back(ctrlPin);
    }

    m_comp.parameters["inputs"] = std::to_string(count);
    accept();
}

// ─── 2. C-Script Code Editor Dialog ───
CScriptEditorDialogQt::CScriptEditorDialogQt(ComponentInstance& comp, QWidget* parent)
    : QDialog(parent), m_comp(comp)
{
    setWindowTitle(QString("Edit C-Script Logic [%1]").arg(QString::fromStdString(comp.id)));
    resize(600, 450);

    QVBoxLayout* layout = new QVBoxLayout(this);

    QLabel* label = new QLabel("C-Script Execution Logic (C++ Syntax):", this);
    label->setStyleSheet("font-weight: bold; color: #38bdf8;");
    layout->addWidget(label);

    m_codeEdit = new QTextEdit(this);
    m_codeEdit->setFont(QFont("JetBrains Mono", 10));
    m_codeEdit->setStyleSheet("background-color: #0f172a; color: #f8fafc; border: 1px solid #334155;");
    m_codeEdit->setText(QString::fromStdString(comp.parameters["code"]));
    layout->addWidget(m_codeEdit);

    QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    connect(bbox, &QDialogButtonBox::accepted, this, &CScriptEditorDialogQt::onSave);
    connect(bbox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(bbox);
}

void CScriptEditorDialogQt::onSave() {
    m_comp.parameters["code"] = m_codeEdit->toPlainText().toStdString();
    accept();
}

// ─── 3. Unified Probe Editor Dialog ───
ProbeEditorDialogQt::ProbeEditorDialogQt(ComponentInstance& comp, QWidget* parent)
    : QDialog(parent), m_comp(comp)
{
    setWindowTitle("Unified Probe Configuration");
    resize(400, 300);

    QVBoxLayout* layout = new QVBoxLayout(this);
    m_targetEdit = new QLineEdit(QString::fromStdString(comp.parameters["target"]), this);
    layout->addWidget(new QLabel("Probe Target Label:"));
    layout->addWidget(m_targetEdit);

    m_signalList = new QListWidget(this);
    m_signalList->setSelectionMode(QAbstractItemView::MultiSelection);
    m_signalList->addItem("V_out (Voltage)");
    m_signalList->addItem("I_L1 (Current)");
    m_signalList->addItem("P_in (Power)");
    layout->addWidget(new QLabel("Selected Signals to Probe:"));
    layout->addWidget(m_signalList);

    QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    connect(bbox, &QDialogButtonBox::accepted, this, &ProbeEditorDialogQt::onSave);
    connect(bbox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(bbox);
}

void ProbeEditorDialogQt::onSave() {
    m_comp.parameters["target"] = m_targetEdit->text().toStdString();
    accept();
}

// ─── 4. PWM Master Dialog ───
PwmMasterDialogQt::PwmMasterDialogQt(ComponentInstance& comp, QWidget* parent)
    : QDialog(parent), m_comp(comp)
{
    setWindowTitle("PWM Master Topology Settings");
    resize(360, 240);

    QFormLayout* form = new QFormLayout(this);

    m_spinCarriers = new QSpinBox(this);
    m_spinCarriers->setRange(1, 6);
    m_spinCarriers->setValue(std::stoi(comp.parameters["num_carriers"].empty() ? "3" : comp.parameters["num_carriers"]));
    form->addRow("Number of Carriers:", m_spinCarriers);

    m_freqEdit = new QLineEdit(QString::fromStdString(comp.parameters["fc"].empty() ? "10k" : comp.parameters["fc"]), this);
    form->addRow("Carrier Frequency (fc):", m_freqEdit);

    m_commonModCheck = new QCheckBox("Common Modulation Signal", this);
    m_commonModCheck->setChecked(comp.parameters["common_modulation"] == "true");
    form->addRow(m_commonModCheck);

    QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    connect(bbox, &QDialogButtonBox::accepted, this, &PwmMasterDialogQt::onSave);
    connect(bbox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    form->addWidget(bbox);
}

void PwmMasterDialogQt::onSave() {
    m_comp.parameters["num_carriers"] = std::to_string(m_spinCarriers->value());
    m_comp.parameters["fc"] = m_freqEdit->text().toStdString();
    m_comp.parameters["common_modulation"] = m_commonModCheck->isChecked() ? "true" : "false";
    accept();
}

} // namespace CircuitSim
