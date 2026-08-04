#include "MainWindow.hpp"
#include <QMenuBar>
#include <QToolBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDebug>

namespace CircuitSim {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("CircuitSim Pro - Native Qt Edition (Cross-Platform)");
    resize(1400, 900);

    // Apply Slate Dark Theme
    setStyleSheet(R"(
        QMainWindow { background-color: #0f1117; color: #e2e8f0; }
        QMenuBar { background-color: #1e222d; color: #e2e8f0; border-bottom: 1px solid #2d3446; }
        QMenuBar::item:selected { background-color: #2b3245; }
        QMenu { background-color: #1e222d; color: #e2e8f0; border: 1px solid #2d3446; }
        QMenu::item:selected { background-color: #3b82f6; color: #ffffff; }
        QToolBar { background-color: #161922; border-bottom: 1px solid #2d3446; spacing: 8px; padding: 4px; }
        QDockWidget { color: #94a3b8; font-weight: bold; titlebar-close-icon: url(); }
        QDockWidget::title { background: #181b24; padding: 6px; border: 1px solid #282e3d; }
        QListWidget, QTableWidget { background-color: #13151c; color: #e2e8f0; border: 1px solid #282e3d; gridline-color: #242a38; }
        QHeaderView::section { background-color: #1c202b; color: #94a3b8; padding: 4px; border: 1px solid #282e3d; }
        QPushButton { background-color: #2563eb; color: #ffffff; border-radius: 4px; padding: 6px 14px; font-weight: bold; }
        QPushButton:hover { background-color: #3b82f6; }
        QPushButton:pressed { background-color: #1d4ed8; }
    )");

    m_canvas = new SchematicView(this);
    setCentralWidget(m_canvas);

    connect(m_canvas, &SchematicView::componentSelected, this, &MainWindow::onComponentSelected);

    createMenuBar();
    createToolBar();
    createPaletteDock();
    createPropertyDock();
    createScopeDock();

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::updateSimTime);
    m_timer->start(50);

    loadTemplate("1ph_inverter_hysteresis");
}

void MainWindow::createMenuBar() {
    QMenu* fileMenu = menuBar()->addMenu("&File");
    fileMenu->addAction("New Workspace", this, &MainWindow::newWorkspace, QKeySequence::New);
    fileMenu->addAction("Open Schematic (.json)", this, &MainWindow::openJsonFile, QKeySequence::Open);
    fileMenu->addAction("Save Schematic (.json)", this, &MainWindow::saveJsonFile, QKeySequence::Save);
    fileMenu->addSeparator();
    fileMenu->addAction("Export Vector Image (.svg)", [this]() {
        QString fileName = QFileDialog::getSaveFileName(this, "Export SVG Schematic", "", "SVG Files (*.svg)");
        if (!fileName.isEmpty()) m_canvas->exportSVG(fileName);
    });
    fileMenu->addSeparator();
    fileMenu->addAction("Exit", this, &QWidget::close, QKeySequence::Quit);

    QMenu* editMenu = menuBar()->addMenu("&Edit");
    editMenu->addAction("Copy", m_canvas, &SchematicView::copySelected, QKeySequence::Copy);
    editMenu->addAction("Paste", m_canvas, &SchematicView::pasteSelected, QKeySequence::Paste);
    editMenu->addAction("Duplicate", m_canvas, &SchematicView::duplicateSelected, QKeySequence("Ctrl+D"));
    editMenu->addSeparator();
    editMenu->addAction("Flip Horizontally (H)", m_canvas, &SchematicView::flipHorizontal);
    editMenu->addAction("Flip Vertically (V)", m_canvas, &SchematicView::flipVertical);

    QMenu* tmplMenu = menuBar()->addMenu("&Templates");
    tmplMenu->addAction("Single-Phase Inverter Hysteresis", [this]() { loadTemplate("1ph_inverter_hysteresis"); });

    QMenu* simMenu = menuBar()->addMenu("&Simulation");
    simMenu->addAction("Start Simulation", this, &MainWindow::startSimulation);
    simMenu->addAction("Pause Simulation", this, &MainWindow::pauseSimulation);
    simMenu->addAction("Reset Simulation", this, &MainWindow::resetSimulation);
}

void MainWindow::createToolBar() {
    QToolBar* toolBar = addToolBar("Simulation Controls");

    QPushButton* btnPlay = new QPushButton("PLAY", this);
    btnPlay->setStyleSheet("background-color: #10b981; color: white;");
    connect(btnPlay, &QPushButton::clicked, this, &MainWindow::startSimulation);
    toolBar->addWidget(btnPlay);

    QPushButton* btnPause = new QPushButton("PAUSE", this);
    btnPause->setStyleSheet("background-color: #f59e0b; color: white;");
    connect(btnPause, &QPushButton::clicked, this, &MainWindow::pauseSimulation);
    toolBar->addWidget(btnPause);

    QPushButton* btnReset = new QPushButton("RESET", this);
    btnReset->setStyleSheet("background-color: #ef4444; color: white;");
    connect(btnReset, &QPushButton::clicked, this, &MainWindow::resetSimulation);
    toolBar->addWidget(btnReset);

    toolBar->addSeparator();
    m_simTimeLabel = new QLabel(" Sim Time: 0.00000 s  ", this);
    m_simTimeLabel->setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 13px;");
    toolBar->addWidget(m_simTimeLabel);
}

void MainWindow::createPaletteDock() {
    QDockWidget* dock = new QDockWidget("Component Palette", this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    QListWidget* list = new QListWidget(dock);
    list->addItem("Resistor (R)");
    list->addItem("Capacitor (C)");
    list->addItem("Inductor (L)");
    list->addItem("DC Voltage Source (V)");
    list->addItem("AC Voltage Source (AC_V)");
    list->addItem("Current Source (I)");
    list->addItem("Diode (D)");
    list->addItem("MOSFET Switch");
    list->addItem("Switch (S)");
    list->addItem("Ground (GND)");
    list->addItem("Gain Block (K)");
    list->addItem("PID Controller");
    list->addItem("Comparator");
    list->addItem("PWM Generator");
    list->addItem("Triangle Carrier");
    list->addItem("Summing Junction");
    list->addItem("C-Script Block");

    connect(list, &QListWidget::itemDoubleClicked, [this](QListWidgetItem* item) {
        QString text = item->text();
        ComponentInstance comp;
        comp.id = "comp_" + std::to_string(rand() % 1000);
        comp.x = 0; comp.y = 0;

        if (text.startsWith("Resistor")) { comp.type = ComponentType::Resistor; comp.rawTypeStr = "R"; comp.label = "Resistor"; }
        else if (text.startsWith("Capacitor")) { comp.type = ComponentType::Capacitor; comp.rawTypeStr = "C"; comp.label = "Capacitor"; }
        else if (text.startsWith("Inductor")) { comp.type = ComponentType::Inductor; comp.rawTypeStr = "L"; comp.label = "Inductor"; }
        else if (text.startsWith("DC Voltage")) { comp.type = ComponentType::VoltageSource; comp.rawTypeStr = "V"; comp.label = "DC Source"; }
        else if (text.startsWith("AC Voltage")) { comp.type = ComponentType::ACVoltageSource; comp.rawTypeStr = "AC_V"; comp.label = "AC Source"; }
        else if (text.startsWith("Diode")) { comp.type = ComponentType::Diode; comp.rawTypeStr = "D"; comp.label = "Diode"; }
        else if (text.startsWith("MOSFET")) { comp.type = ComponentType::MOSFET; comp.rawTypeStr = "MOSFET"; comp.label = "MOSFET"; }
        else if (text.startsWith("Ground")) { comp.type = ComponentType::Unknown; comp.rawTypeStr = "GND"; comp.label = "GND"; }
        else { comp.type = ComponentType::Resistor; comp.rawTypeStr = "R"; comp.label = "Resistor"; }

        m_canvas->addComponent(comp);
    });

    dock->setWidget(list);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
}

void MainWindow::createPropertyDock() {
    QDockWidget* dock = new QDockWidget("Property Inspector", this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    QWidget* container = new QWidget(dock);
    QVBoxLayout* layout = new QVBoxLayout(container);

    m_selectedLabel = new QLabel("Select a component on canvas to inspect", container);
    m_selectedLabel->setStyleSheet("color: #94a3b8; font-style: italic;");
    layout->addWidget(m_selectedLabel);

    m_propTable = new QTableWidget(0, 2, container);
    m_propTable->setHorizontalHeaderLabels({"Attribute", "Value"});
    m_propTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    layout->addWidget(m_propTable);

    container->setLayout(layout);
    dock->setWidget(container);
    addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::createScopeDock() {
    QDockWidget* dock = new QDockWidget("Real-Time Oscilloscope Waveforms", this);
    dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

    m_scopeWidget = new OscilloscopeWidget(dock);
    m_scopeWidget->setSimulator(&m_simulator);
    dock->setWidget(m_scopeWidget);

    addDockWidget(Qt::BottomDockWidgetArea, dock);
}

void MainWindow::newWorkspace() {
    m_canvas->setCircuit(CircuitDesign());
    m_simulator.loadCircuit(CircuitDesign());
}

void MainWindow::openJsonFile() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open JSON Schematic", "", "JSON Files (*.json)");
    if (!fileName.isEmpty()) openJsonSchematic(fileName);
}

void MainWindow::saveJsonFile() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save JSON Schematic", "", "JSON Files (*.json)");
    if (!fileName.isEmpty()) saveJsonSchematic(fileName);
}

void MainWindow::openJsonSchematic(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return;

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    QJsonObject obj = doc.object();
    file.close();

    CircuitDesign cd;
    QJsonArray compArray = obj["components"].toArray();
    for (auto val : compArray) {
        QJsonObject cObj = val.toObject();
        ComponentInstance comp;
        comp.id = cObj["id"].toString().toStdString();
        comp.rawTypeStr = cObj["type"].toString().toStdString();
        comp.label = cObj["label"].toString().toStdString();
        comp.x = cObj["x"].toDouble();
        comp.y = cObj["y"].toDouble();
        comp.rotation = cObj["rotation"].toInt();

        QJsonObject pObj = cObj["parameters"].toObject();
        for (auto key : pObj.keys()) {
            comp.parameters[key.toStdString()] = pObj[key].toString().toStdString();
        }
        cd.components.push_back(comp);
    }

    QJsonArray wireArray = obj["wires"].toArray();
    for (auto val : wireArray) {
        QJsonObject wObj = val.toObject();
        WireInstance wire;
        wire.id = wObj["id"].toString().toStdString();
        wire.from.compId = wObj["from"].toObject()["compId"].toString().toStdString();
        wire.from.terminal = wObj["from"].toObject()["terminal"].toString().toStdString();
        wire.to.compId = wObj["to"].toObject()["compId"].toString().toStdString();
        wire.to.terminal = wObj["to"].toObject()["terminal"].toString().toStdString();
        cd.wires.push_back(wire);
    }

    m_canvas->setCircuit(cd);
    m_simulator.loadCircuit(cd);
}

void MainWindow::saveJsonSchematic(const QString& filePath) {
    CircuitDesign cd = m_canvas->getCircuit();
    QJsonObject rootObj;

    QJsonArray compArray;
    for (const auto& comp : cd.components) {
        QJsonObject cObj;
        cObj["id"] = QString::fromStdString(comp.id);
        cObj["type"] = QString::fromStdString(comp.rawTypeStr);
        cObj["label"] = QString::fromStdString(comp.label);
        cObj["x"] = comp.x;
        cObj["y"] = comp.y;
        cObj["rotation"] = comp.rotation;

        QJsonObject pObj;
        for (const auto& pair : comp.parameters) {
            pObj[QString::fromStdString(pair.first)] = QString::fromStdString(pair.second);
        }
        cObj["parameters"] = pObj;
        compArray.append(cObj);
    }
    rootObj["components"] = compArray;

    QJsonArray wireArray;
    for (const auto& wire : cd.wires) {
        QJsonObject wObj;
        wObj["id"] = QString::fromStdString(wire.id);

        QJsonObject fObj;
        fObj["type"] = "pin";
        fObj["compId"] = QString::fromStdString(wire.from.compId);
        fObj["terminal"] = QString::fromStdString(wire.from.terminal);
        wObj["from"] = fObj;

        QJsonObject tObj;
        tObj["type"] = "pin";
        tObj["compId"] = QString::fromStdString(wire.to.compId);
        tObj["terminal"] = QString::fromStdString(wire.to.terminal);
        wObj["to"] = tObj;

        wireArray.append(wObj);
    }
    rootObj["wires"] = wireArray;

    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(QJsonDocument(rootObj).toJson());
        file.close();
    }
}

void MainWindow::loadTemplate(const QString& name) {
    CircuitDesign cd;
    if (name == "1ph_inverter_hysteresis") {
        ComponentInstance v1; v1.id = "V1"; v1.label = "DC 24V"; v1.rawTypeStr = "V"; v1.x = -200; v1.y = 0; cd.components.push_back(v1);
        ComponentInstance m1; m1.id = "M1"; m1.label = "MOSFET 1"; m1.rawTypeStr = "MOSFET"; m1.x = 0; m1.y = -80; cd.components.push_back(m1);
        ComponentInstance m2; m2.id = "M2"; m2.label = "MOSFET 2"; m2.rawTypeStr = "MOSFET"; m2.x = 0; m2.y = 80; cd.components.push_back(m2);
        ComponentInstance l1; l1.id = "L1"; l1.label = "L1 (10mH)"; l1.rawTypeStr = "L"; l1.x = 160; l1.y = 0; cd.components.push_back(l1);
        ComponentInstance r1; r1.id = "R1"; r1.label = "R1 (10 Ohm)"; r1.rawTypeStr = "R"; r1.x = 300; r1.y = 0; cd.components.push_back(r1);
        ComponentInstance gnd; gnd.id = "GND1"; gnd.label = "GND"; gnd.rawTypeStr = "GND"; gnd.x = -200; gnd.y = 120; cd.components.push_back(gnd);

        WireInstance w1; w1.from = {"V1", "A"}; w1.to = {"M1", "D"}; cd.wires.push_back(w1);
        WireInstance w2; w2.from = {"M1", "S"}; w2.to = {"M2", "D"}; cd.wires.push_back(w2);
        WireInstance w3; w3.from = {"M1", "S"}; w3.to = {"L1", "A"}; cd.wires.push_back(w3);
        WireInstance w4; w4.from = {"L1", "B"}; w4.to = {"R1", "A"}; cd.wires.push_back(w4);
    }

    m_canvas->setCircuit(cd);
    m_simulator.loadCircuit(cd);
}

void MainWindow::startSimulation() { m_simulator.runAsync(); }
void MainWindow::pauseSimulation() { m_simulator.pause(); }
void MainWindow::resetSimulation() { m_simulator.reset(); }

void MainWindow::updateSimTime() {
    m_simTimeLabel->setText(QString(" Sim Time: %1 s ").arg(m_simulator.getCurrentTime(), 0, 'f', 5));
}

void MainWindow::onComponentSelected(const ComponentInstance* comp) {
    if (!comp) {
        m_selectedLabel->setText("Select a component on canvas to inspect");
        m_propTable->setRowCount(0);
        return;
    }

    m_selectedLabel->setText(QString("Selected: %1 (%2)").arg(QString::fromStdString(comp->id), QString::fromStdString(comp->label)));
    m_propTable->setRowCount(3);

    m_propTable->setItem(0, 0, new QTableWidgetItem("ID"));
    m_propTable->setItem(0, 1, new QTableWidgetItem(QString::fromStdString(comp->id)));

    m_propTable->setItem(1, 0, new QTableWidgetItem("Type"));
    m_propTable->setItem(1, 1, new QTableWidgetItem(QString::fromStdString(comp->rawTypeStr)));

    m_propTable->setItem(2, 0, new QTableWidgetItem("Label"));
    m_propTable->setItem(2, 1, new QTableWidgetItem(QString::fromStdString(comp->label)));
}

} // namespace CircuitSim
