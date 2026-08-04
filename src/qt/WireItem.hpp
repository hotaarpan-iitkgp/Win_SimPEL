#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <QVector>
#include <QPointF>
#include "../engine/Components.hpp"

namespace CircuitSim {

class WireItem : public QGraphicsItem {
public:
    enum { Type = UserType + 2 };

    WireItem(const WireInstance& instance, QGraphicsItem* parent = nullptr);

    int type() const override { return Type; }
    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    const WireInstance& getInstance() const { return m_instance; }
    void setEndpoints(const QPointF& start, const QPointF& end);

private:
    WireInstance m_instance;
    QPointF m_start;
    QPointF m_end;
    QVector<QPointF> m_pathPoints;
    
    void updatePath();
};

} // namespace CircuitSim
