#include "OscilloscopeWidget.hpp"
#include <QPainter>
#include <QPainterPath>
#include <cmath>

namespace CircuitSim {

OscilloscopeWidget::OscilloscopeWidget(QWidget* parent)
    : QWidget(parent)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &OscilloscopeWidget::updateWaveforms);
    m_timer->start(30); // 33 FPS real-time graph update
}

void OscilloscopeWidget::setSimulator(CircuitSimulator* simulator) {
    m_simulator = simulator;
}

void OscilloscopeWidget::updateWaveforms() {
    if (m_simulator && m_simulator->getIsRunning()) {
        update();
    }
}

void OscilloscopeWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Oscilloscope Screen Background (Dark Slate)
    painter.fillRect(rect(), QColor(10, 12, 18));

    // Grid lines
    painter.setPen(QPen(QColor(30, 36, 48), 1, Qt::DashLine));
    int gridX = 8, gridY = 6;
    for (int i = 1; i < gridX; ++i) {
        int x = width() * i / gridX;
        painter.drawLine(x, 0, x, height());
    }
    for (int j = 1; j < gridY; ++j) {
        int y = height() * j / gridY;
        painter.drawLine(0, y, width(), y);
    }

    if (!m_simulator) return;

    TelemetryData telemetry = m_simulator->getTelemetryCopy();
    const auto& timePoints = telemetry.timeHistory;
    const auto& channels = telemetry.voltages;

    if (timePoints.empty() || channels.empty()) {
        painter.setPen(QColor(120, 135, 155));
        painter.drawText(rect(), Qt::AlignCenter, "Oscilloscope: Press PLAY to start real-time signal simulation");
        return;
    }

    double tMin = timePoints.front();
    double tMax = timePoints.back();
    if (tMax <= tMin) tMax = tMin + 1.0;

    QList<QColor> colors = { QColor(0, 230, 120), QColor(0, 200, 255), QColor(255, 180, 0), QColor(255, 90, 120) };
    int colorIdx = 0;

    for (auto it = channels.begin(); it != channels.end(); ++it) {
        const auto& data = it->second;
        if (data.empty()) continue;

        double vMin = -5.0, vMax = 25.0;
        QPainterPath path;

        int n = std::min((int)timePoints.size(), (int)data.size());
        for (int i = 0; i < n; ++i) {
            double t = timePoints[i];
            double v = data[i];

            double px = (t - tMin) / (tMax - tMin) * width();
            double py = height() - (v - vMin) / (vMax - vMin) * height();

            if (i == 0) path.moveTo(px, py);
            else path.lineTo(px, py);
        }

        painter.setPen(QPen(colors[colorIdx % colors.size()], 2.0));
        painter.drawPath(path);
        colorIdx++;
    }
}

} // namespace CircuitSim
