#pragma once

#include <QMainWindow>
#include <QDockWidget>
#include <QListWidget>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QTimer>
#include "SchematicView.hpp"
#include "OscilloscopeWidget.hpp"
#include "../engine/CircuitSimulator.hpp"

namespace CircuitSim {

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void newWorkspace();
    void openJsonFile();
    void saveJsonFile();
    void loadTemplate(const QString& name);
    void startSimulation();
    void pauseSimulation();
    void resetSimulation();
    void updateSimTime();
    void onComponentSelected(const ComponentInstance* comp);

private:
    SchematicView* m_canvas;
    OscilloscopeWidget* m_scopeWidget;
    CircuitSimulator m_simulator;
    
    QTableWidget* m_propTable;
    QLabel* m_selectedLabel;
    QLabel* m_simTimeLabel;
    QTimer* m_timer;
    
    void createMenuBar();
    void createToolBar();
    void createPaletteDock();
    void createPropertyDock();
    void createScopeDock();
    
    void openJsonSchematic(const QString& filePath);
    void saveJsonSchematic(const QString& filePath);
};

} // namespace CircuitSim
