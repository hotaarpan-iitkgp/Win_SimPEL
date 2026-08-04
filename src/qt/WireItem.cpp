#include "WireItem.hpp"
#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionGraphicsItem>
#include <cmath>

namespace CircuitSim {

WireItem::WireItem(const WireInstance& instance, QGraphicsItem* parent)
    : QGraphicsItem(parent), m_instance(instance)
{
    setFlags(ItemIsSelectable);
    setZValue(-1.0); // Wires drawn behind component bodies
}

void WireItem::setEndpoints(const QPointF& start, const QPointF& end) {
    prepareGeometryChange();
    m_start = start;
    m_end = end;
    updatePath();
}

void WireItem::updatePath() {
    m_pathPoints.clear();
    m_pathPoints.append(m_start);
    
    // Calculate orthogonal intermediate point (L-shaped or Z-shaped)
    QPointF mid((m_start.x() + m_end.x()) * 0.5, m_start.y());
    m_pathPoints.append(mid);
    m_pathPoints.append(QPointF(mid.x(), m_end.y()));
    m_pathPoints.append(m_end);
}

QRectF WireItem::boundingRect() const {
    qreal minX = qMin(m_start.x(), m_end.x()) - 10;
    qreal minY = qMin(m_start.y(), m_end.y()) - 10;
    qreal maxX = qMax(m_start.x(), m_end.x()) + 10;
    qreal maxY = qMax(m_start.y(), m_end.y()) + 10;
    return QRectF(minX, minY, maxX - minX, maxY - minY);
}

void WireItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);

    bool isSelected = (option->state & QStyle::State_Selected);
    QColor color = isSelected ? QColor(255, 180, 0) : QColor(0, 230, 120);
    QPen pen(color, isSelected ? 3.0 : 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);

    QPainterPath path;
    if (!m_pathPoints.isEmpty()) {
        path.moveTo(m_pathPoints[0]);
        for (int i = 1; i < m_pathPoints.size(); ++i) {
            path.lineTo(m_pathPoints[i]);
        }
    }
    painter->drawPath(path);

    // Junction dots at connection endpoints
    painter->setPen(Qt::NoPen);
    painter->setBrush(color);
    painter->drawEllipse(m_start, 3.5, 3.5);
    painter->drawEllipse(m_end, 3.5, 3.5);
}

} // namespace CircuitSim
