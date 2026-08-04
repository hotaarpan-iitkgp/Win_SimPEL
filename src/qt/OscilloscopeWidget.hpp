#pragma once

#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QVector>
#include <QPointF>
#include "../engine/CircuitSimulator.hpp"

namespace CircuitSim {

class OscilloscopeWidget : public QWidget {
    Q_OBJECT
public:
    explicit OscilloscopeWidget(QWidget* parent = nullptr);

    void setSimulator(CircuitSimulator* simulator);

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void updateWaveforms();

private:
    CircuitSimulator* m_simulator = nullptr;
    QTimer* m_timer;
};

} // namespace CircuitSim
