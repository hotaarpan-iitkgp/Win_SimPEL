#pragma once

#include <QGraphicsItem>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QString>
#include <QVector>
#include <QMap>
#include "../engine/Components.hpp"

namespace CircuitSim {

struct QtTerminalPin {
    QString name;
    QPointF relPos;  // Relative to component center (0,0)
    QPointF dir;     // Routing direction vector
    bool isOutput = false;
    bool isCtrl = false;
    QString opSign;
};

class ComponentItem : public QGraphicsItem {
public:
    enum { Type = UserType + 1 };

    ComponentItem(const ComponentInstance& instance, QGraphicsItem* parent = nullptr);

    int type() const override { return Type; }
    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    const ComponentInstance& getInstance() const { return m_instance; }
    ComponentInstance& getInstanceRef() { return m_instance; }
    void updateFromInstance(const ComponentInstance& inst);

    QPointF getTerminalScenePos(const QString& pinName) const;
    QVector<QtTerminalPin> getTerminals() const { return m_terminals; }
    QString getHoveredTerminal(const QPointF& scenePos) const;

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

private:
    ComponentInstance m_instance;
    QVector<QtTerminalPin> m_terminals;
    
    void setupTerminals();
    void drawShape(QPainter* painter);
};

} // namespace CircuitSim
