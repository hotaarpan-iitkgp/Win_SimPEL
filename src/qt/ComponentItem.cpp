#include "ComponentItem.hpp"
#include <QPainter>
#include <QGraphicsScene>
#include <QStyleOptionGraphicsItem>
#include <QtMath>
#include <cmath>

namespace CircuitSim {

ComponentItem::ComponentItem(const ComponentInstance& instance, QGraphicsItem* parent)
    : QGraphicsItem(parent), m_instance(instance)
{
    setFlags(ItemIsSelectable | ItemIsMovable | ItemSendsGeometryChanges);
    setAcceptHoverEvents(true);
    setPos(instance.x, instance.y);
    setRotation(instance.rotation);
    setupTerminals();
}

void ComponentItem::updateFromInstance(const ComponentInstance& inst) {
    m_instance = inst;
    setPos(inst.x, inst.y);
    setRotation(inst.rotation);
    setupTerminals();
    update();
}

void ComponentItem::setupTerminals() {
    m_terminals.clear();
    const std::string& t = m_instance.rawTypeStr;

    if (t == "R" || t == "L" || t == "C" || t == "V" || t == "I" || t == "D" || t == "AC_V") {
        m_terminals.append({"A", QPointF(0, -40), QPointF(0, -1)});
        m_terminals.append({"B", QPointF(0, 40), QPointF(0, 1)});
    } else if (t == "VM" || t == "AM") {
        m_terminals.append({"A", QPointF(0, -40), QPointF(0, -1)});
        m_terminals.append({"B", QPointF(0, 40), QPointF(0, 1)});
        m_terminals.append({"Out", QPointF(20, 0), QPointF(1, 0), true});
    } else if (t == "MOSFET" || t == "vg-FET") {
        m_terminals.append({"D", QPointF(0, -40), QPointF(0, -1)});
        m_terminals.append({"S", QPointF(0, 40), QPointF(0, 1)});
        m_terminals.append({"G", QPointF(-20, 0), QPointF(-1, 0), false, true});
    } else if (t == "S") {
        m_terminals.append({"A", QPointF(0, -40), QPointF(0, -1)});
        m_terminals.append({"B", QPointF(0, 40), QPointF(0, 1)});
        m_terminals.append({"Ctrl", QPointF(-20, 0), QPointF(-1, 0), false, true});
    } else if (t == "GND") {
        m_terminals.append({"Gnd", QPointF(0, -20), QPointF(0, -1)});
    } else if (t == "CONST" || t == "TRI") {
        m_terminals.append({"Out", QPointF(20, 0), QPointF(1, 0), true});
    } else if (t == "GAIN" || t == "PID" || t == "PWM" || t == "FCN" || t == "NOT") {
        m_terminals.append({"In", QPointF(-20, 0), QPointF(-1, 0)});
        m_terminals.append({"Out", QPointF(20, 0), QPointF(1, 0), true});
    } else if (t == "SUM" || t == "PROD" || t == "COMP" || t == "AND" || t == "OR") {
        m_terminals.append({"A", QPointF(-20, -20), QPointF(-1, 0)});
        m_terminals.append({"B", QPointF(-20, 20), QPointF(-1, 0)});
        m_terminals.append({"Out", QPointF(20, 0), QPointF(1, 0), true});
    } else if (t == "CSCRIPT") {
        m_terminals.append({"In1", QPointF(-80, 0), QPointF(-1, 0)});
        m_terminals.append({"Out1", QPointF(80, -30), QPointF(1, 0), true});
        m_terminals.append({"Out2", QPointF(80, -10), QPointF(1, 0), true});
        m_terminals.append({"Out3", QPointF(80, 10), QPointF(1, 0), true});
        m_terminals.append({"Out4", QPointF(80, 30), QPointF(1, 0), true});
    }
}

QRectF ComponentItem::boundingRect() const {
    return QRectF(-90, -70, 180, 140);
}

QPointF ComponentItem::getTerminalScenePos(const QString& pinName) const {
    for (const auto& pin : m_terminals) {
        if (pin.name == pinName) {
            return mapToScene(pin.relPos);
        }
    }
    return mapToScene(QPointF(0, 0));
}

QString ComponentItem::getHoveredTerminal(const QPointF& scenePos) const {
    QPointF localPos = mapFromScene(scenePos);
    for (const auto& pin : m_terminals) {
        if (QLineF(localPos, pin.relPos).length() < 12.0) {
            return pin.name;
        }
    }
    return QString();
}

QVariant ComponentItem::itemChange(GraphicsItemChange change, const QVariant& value) {
    if (change == ItemPositionHasChanged) {
        m_instance.x = pos().x();
        m_instance.y = pos().y();
    }
    return QGraphicsItem::itemChange(change, value);
}

void ComponentItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);

    bool isSelected = (option->state & QStyle::State_Selected);
    QColor symbolColor = isSelected ? QColor(255, 180, 0) : QColor(200, 215, 235);
    QPen pen(symbolColor, 2.0);
    painter->setPen(pen);

    if (isSelected) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(QColor(255, 180, 0, 35));
        painter->drawRoundedRect(QRectF(-25, -45, 50, 90), 6, 6);
        painter->setPen(pen);
        painter->setBrush(Qt::NoBrush);
    }

    drawShape(painter);

    // Draw Labels
    QString labelText = m_instance.label.empty() ? QString::fromStdString(m_instance.id) : QString::fromStdString(m_instance.label);
    painter->setFont(QFont("Inter", 8, QFont::Medium));
    painter->setPen(QColor(180, 195, 215));
    painter->drawText(QRectF(-80, 44, 160, 20), Qt::AlignCenter, labelText);

    // Draw Terminal Dots
    for (const auto& pin : m_terminals) {
        QColor pinColor = pin.isCtrl ? QColor(255, 100, 100) : (pin.isOutput ? QColor(50, 220, 130) : QColor(0, 200, 255));
        painter->setPen(Qt::NoPen);
        painter->setBrush(pinColor);
        painter->drawEllipse(pin.relPos, 4.0, 4.0);
    }
}

void ComponentItem::drawShape(QPainter* painter) {
    const std::string& t = m_instance.rawTypeStr;

    if (t == "R") { // Resistor Zigzag
        QPainterPath path;
        path.moveTo(0, -40); path.lineTo(0, -20);
        path.lineTo(-10, -15); path.lineTo(10, -9);
        path.lineTo(-10, -3);  path.lineTo(10, 3);
        path.lineTo(-10, 9);   path.lineTo(10, 15);
        path.lineTo(0, 20);    path.lineTo(0, 40);
        painter->drawPath(path);
    } else if (t == "L") { // Inductor Coils
        painter->drawLine(0, -40, 0, -20);
        for (int i = 0; i < 3; ++i) {
            double cy = -13.3 + i * 13.3;
            QPainterPath path;
            path.moveTo(0, cy - 6.7);
            path.cubicTo(-14, cy - 6.7, -14, cy + 6.7, 0, cy + 6.7);
            painter->drawPath(path);
        }
        painter->drawLine(0, 20, 0, 40);
    } else if (t == "C") { // Capacitor Plates
        painter->drawLine(0, -40, 0, -5);
        painter->drawLine(-15, -5, 15, -5);
        painter->drawLine(-15, 5, 15, 5);
        painter->drawLine(0, 5, 0, 40);
    } else if (t == "V") { // DC Source
        painter->drawLine(0, -40, 0, -16);
        painter->drawLine(0, 16, 0, 40);
        painter->drawEllipse(QPointF(0, 0), 16, 16);
        painter->drawLine(-3, -7, 3, -7); painter->drawLine(0, -10, 0, -4);
        painter->drawLine(-3, 7, 3, 7);
    } else if (t == "AC_V") { // AC Source
        painter->drawLine(0, -40, 0, -16);
        painter->drawLine(0, 16, 0, 40);
        painter->drawEllipse(QPointF(0, 0), 16, 16);
        QPainterPath path;
        path.moveTo(-8, 0);
        path.cubicTo(-4, -8, 0, -8, 0, 0);
        path.cubicTo(0, 8, 4, 8, 8, 0);
        painter->drawPath(path);
    } else if (t == "MOSFET") { // MOSFET Switch with Body Diode
        painter->drawLine(0, -40, 0, -15);
        painter->drawLine(0, 15, 0, 40);
        painter->drawLine(-5, -15, -5, 15);
        painter->drawLine(-5, 0, 0, 0);
        painter->drawLine(0, -15, 0, -10); painter->drawLine(0, 15, 0, 10);
        painter->drawLine(-10, -15, -10, 15);
        painter->drawLine(-20, 0, -10, 0);
        // Antiparallel Diode
        QPolygonF tri; tri << QPointF(7, 6) << QPointF(17, 6) << QPointF(12, -6);
        painter->drawPolygon(tri);
        painter->drawLine(7, -6, 17, -6);
        painter->drawLine(0, -15, 12, -15); painter->drawLine(12, -15, 12, -6);
        painter->drawLine(12, 6, 12, 15); painter->drawLine(12, 15, 0, 15);
    } else if (t == "D") { // Diode
        painter->drawLine(0, -40, 0, -10);
        QPolygonF tri; tri << QPointF(-15, -10) << QPointF(15, -10) << QPointF(0, 12);
        painter->drawPolygon(tri);
        painter->drawLine(-15, 12, 15, 12);
        painter->drawLine(0, 12, 0, 40);
    } else if (t == "GND") { // Ground
        painter->drawLine(0, -20, 0, 0);
        painter->drawLine(-12, 0, 12, 0);
        painter->drawLine(-8, 6, 8, 6);
        painter->drawLine(-4, 12, 4, 12);
    } else { // Generic Block
        painter->drawRoundedRect(QRectF(-20, -20, 40, 40), 4, 4);
        painter->drawText(QRectF(-20, -10, 40, 20), Qt::AlignCenter, QString::fromStdString(t));
    }
}

} // namespace CircuitSim
