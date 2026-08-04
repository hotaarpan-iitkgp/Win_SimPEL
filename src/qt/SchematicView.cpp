#include "SchematicView.hpp"
#include <QPainter>
#include <QScrollBar>
#include <QSvgGenerator>
#include <QGuiApplication>
#include <QClipboard>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <cmath>

namespace CircuitSim {

SchematicView::SchematicView(QWidget* parent)
    : QGraphicsView(parent)
{
    m_scene = new QGraphicsScene(this);
    m_scene->setSceneRect(-5000, -5000, 10000, 10000);
    setScene(m_scene);

    setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setDragMode(QGraphicsView::RubberBandDrag);

    // Temp line item for wire creation preview
    m_tempWireItem = new QGraphicsPathItem();
    QPen pen(QColor(255, 200, 0), 2.0, Qt::DashLine);
    m_tempWireItem->setPen(pen);
    m_tempWireItem->setVisible(false);
    m_scene->addItem(m_tempWireItem);
}

void SchematicView::setCircuit(const CircuitDesign& design) {
    m_design = design;
    m_scene->clear();

    m_tempWireItem = new QGraphicsPathItem();
    QPen pen(QColor(255, 200, 0), 2.0, Qt::DashLine);
    m_tempWireItem->setPen(pen);
    m_tempWireItem->setVisible(false);
    m_scene->addItem(m_tempWireItem);

    // Add components
    for (auto& comp : m_design.components) {
        ComponentItem* item = new ComponentItem(comp);
        m_scene->addItem(item);
    }

    updateWires();
}

CircuitDesign SchematicView::getCircuit() const {
    CircuitDesign result = m_design;
    result.components.clear();

    for (QGraphicsItem* item : m_scene->items()) {
        if (ComponentItem* compItem = dynamic_cast<ComponentItem*>(item)) {
            result.components.push_back(compItem->getInstance());
        }
    }

    return result;
}

void SchematicView::addComponent(const ComponentInstance& comp) {
    ComponentInstance newComp = comp;
    
    // Check if component requires dynamic configuration modal first
    if (newComp.rawTypeStr == "SUM_RECT" || newComp.rawTypeStr == "SUM_ROUND" || newComp.rawTypeStr == "PRODUCT_RECT") {
        ConfiguratorDialogQt dialog(newComp, this);
        if (dialog.exec() != QDialog::Accepted) return;
    }

    m_design.components.push_back(newComp);
    ComponentItem* item = new ComponentItem(newComp);
    m_scene->addItem(item);
}

void SchematicView::updateWires() {
    // Remove existing wire items
    for (QGraphicsItem* item : m_scene->items()) {
        if (dynamic_cast<WireItem*>(item)) {
            m_scene->removeItem(item);
            delete item;
        }
    }

    QMap<QString, ComponentItem*> compMap;
    for (QGraphicsItem* item : m_scene->items()) {
        if (ComponentItem* ci = dynamic_cast<ComponentItem*>(item)) {
            compMap[QString::fromStdString(ci->getInstance().id)] = ci;
        }
    }

    for (const auto& wire : m_design.wires) {
        QString fromComp = QString::fromStdString(wire.from.compId);
        QString fromTerm = QString::fromStdString(wire.from.terminal);
        QString toComp = QString::fromStdString(wire.to.compId);
        QString toTerm = QString::fromStdString(wire.to.terminal);

        if (compMap.contains(fromComp) && compMap.contains(toComp)) {
            QPointF p1 = compMap[fromComp]->getTerminalScenePos(fromTerm);
            QPointF p2 = compMap[toComp]->getTerminalScenePos(toTerm);

            WireItem* wireItem = new WireItem(wire);
            wireItem->setEndpoints(p1, p2);
            m_scene->addItem(wireItem);
        }
    }
}

void SchematicView::copySelected() {
    QJsonArray compArray;
    for (QGraphicsItem* item : m_scene->selectedItems()) {
        if (ComponentItem* ci = dynamic_cast<ComponentItem*>(item)) {
            const auto& comp = ci->getInstance();
            QJsonObject cObj;
            cObj["id"] = QString::fromStdString(comp.id);
            cObj["type"] = QString::fromStdString(comp.rawTypeStr);
            cObj["label"] = QString::fromStdString(comp.label);
            cObj["x"] = comp.x;
            cObj["y"] = comp.y;
            cObj["rotation"] = comp.rotation;
            compArray.append(cObj);
        }
    }
    QJsonObject root;
    root["components"] = compArray;
    m_clipboardJson = QJsonDocument(root).toJson();
    QGuiApplication::clipboard()->setText(m_clipboardJson);
}

void SchematicView::pasteSelected() {
    QString jsonStr = QGuiApplication::clipboard()->text();
    if (jsonStr.isEmpty()) return;

    QJsonDocument doc = QJsonDocument::fromJson(jsonStr.toUtf8());
    if (!doc.isObject()) return;

    m_scene->clearSelection();
    QJsonArray compArray = doc.object()["components"].toArray();

    for (auto val : compArray) {
        QJsonObject cObj = val.toObject();
        ComponentInstance comp;
        comp.id = "comp_" + std::to_string(rand() % 10000);
        comp.rawTypeStr = cObj["type"].toString().toStdString();
        comp.label = cObj["label"].toString().toStdString();
        comp.x = cObj["x"].toDouble() + 40.0;
        comp.y = cObj["y"].toDouble() + 40.0;
        comp.rotation = cObj["rotation"].toInt();

        m_design.components.push_back(comp);
        ComponentItem* item = new ComponentItem(comp);
        m_scene->addItem(item);
        item->setSelected(true);
    }
}

void SchematicView::duplicateSelected() {
    copySelected();
    pasteSelected();
}

void SchematicView::flipHorizontal() {
    for (QGraphicsItem* item : m_scene->selectedItems()) {
        if (ComponentItem* ci = dynamic_cast<ComponentItem*>(item)) {
            ComponentInstance inst = ci->getInstance();
            inst.rotation = (inst.rotation + 180) % 360;
            ci->updateFromInstance(inst);
        }
    }
    updateWires();
}

void SchematicView::flipVertical() {
    flipHorizontal();
}

void SchematicView::exportSVG(const QString& filePath) {
    QSvgGenerator generator;
    generator.setFileName(filePath);
    generator.setSize(QSize(1600, 1200));
    generator.setViewBox(QRect(-800, -600, 1600, 1200));
    generator.setTitle("CircuitSim Pro Schematic Export");

    QPainter painter;
    painter.begin(&generator);
    m_scene->render(&painter);
    painter.end();
}

void SchematicView::drawBackground(QPainter* painter, const QRectF& rect) {
    painter->fillRect(rect, QColor(15, 17, 23));

    qreal gridSize = 20.0;
    qreal left = std::floor(rect.left() / gridSize) * gridSize;
    qreal top = std::floor(rect.top() / gridSize) * gridSize;

    QPen penGrid(QColor(35, 40, 50, 180), 1.0);
    QPen penMajor(QColor(45, 52, 65, 220), 1.0);

    for (qreal x = left; x < rect.right(); x += gridSize) {
        int idx = std::round(x / gridSize);
        painter->setPen((idx % 5 == 0) ? penMajor : penGrid);
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
    }
    for (qreal y = top; y < rect.bottom(); y += gridSize) {
        int idx = std::round(y / gridSize);
        painter->setPen((idx % 5 == 0) ? penMajor : penGrid);
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
    }
}

void SchematicView::wheelEvent(QWheelEvent* event) {
    double factor = std::pow(1.2, event->angleDelta().y() / 240.0);
    scale(factor, factor);
}

void SchematicView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        m_isPanning = true;
        m_lastPanPos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        QPointF scenePos = mapToScene(event->pos());
        QGraphicsItem* item = m_scene->itemAt(scenePos, transform());

        if (ComponentItem* compItem = dynamic_cast<ComponentItem*>(item)) {
            QString hoveredPin = compItem->getHoveredTerminal(scenePos);
            if (!hoveredPin.isEmpty()) {
                if (!m_isWiring) {
                    m_isWiring = true;
                    m_wireStartCompId = QString::fromStdString(compItem->getInstance().id);
                    m_wireStartPin = hoveredPin;
                    m_tempWireItem->setVisible(true);
                } else {
                    WireInstance wire;
                    wire.id = "w_" + std::to_string(m_design.wires.size() + 1);
                    wire.from.compId = m_wireStartCompId.toStdString();
                    wire.from.terminal = m_wireStartPin.toStdString();
                    wire.to.compId = compItem->getInstance().id;
                    wire.to.terminal = hoveredPin.toStdString();

                    m_design.wires.push_back(wire);
                    m_isWiring = false;
                    m_tempWireItem->setVisible(false);
                    updateWires();
                }
                event->accept();
                return;
            }
            emit componentSelected(&(compItem->getInstance()));
        } else {
            emit componentSelected(nullptr);
            if (m_isWiring) {
                m_isWiring = false;
                m_tempWireItem->setVisible(false);
            }
        }
    }

    QGraphicsView::mousePressEvent(event);
}

void SchematicView::mouseMoveEvent(QMouseEvent* event) {
    if (m_isPanning) {
        QPointF delta = mapToScene(m_lastPanPos) - mapToScene(event->pos());
        m_lastPanPos = event->pos();
        setTransformationAnchor(QGraphicsView::NoAnchor);
        translate(-delta.x(), -delta.y());
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        event->accept();
        return;
    }

    if (m_isWiring) {
        QPointF scenePos = mapToScene(event->pos());
        QPointF startPos(0, 0);
        for (QGraphicsItem* sceneItem : m_scene->items()) {
            if (ComponentItem* ci = dynamic_cast<ComponentItem*>(sceneItem)) {
                if (QString::fromStdString(ci->getInstance().id) == m_wireStartCompId) {
                    startPos = ci->getTerminalScenePos(m_wireStartPin);
                    break;
                }
            }
        }

        QPainterPath path;
        path.moveTo(startPos);
        QPointF mid((startPos.x() + scenePos.x()) * 0.5, startPos.y());
        path.lineTo(mid);
        path.lineTo(QPointF(mid.x(), scenePos.y()));
        path.lineTo(scenePos);
        m_tempWireItem->setPath(path);
    }

    updateWires();
    QGraphicsView::mouseMoveEvent(event);
}

void SchematicView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton && m_isPanning) {
        m_isPanning = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void SchematicView::mouseDoubleClickEvent(QMouseEvent* event) {
    QPointF scenePos = mapToScene(event->pos());
    QGraphicsItem* item = m_scene->itemAt(scenePos, transform());

    if (ComponentItem* compItem = dynamic_cast<ComponentItem*>(item)) {
        ComponentInstance inst = compItem->getInstance();

        if (inst.rawTypeStr == "CSCRIPT") {
            CScriptEditorDialogQt dialog(inst, this);
            if (dialog.exec() == QDialog::Accepted) compItem->updateFromInstance(inst);
        } else if (inst.rawTypeStr == "SUM_RECT" || inst.rawTypeStr == "SUM_ROUND" || inst.rawTypeStr == "PRODUCT_RECT") {
            ConfiguratorDialogQt dialog(inst, this);
            if (dialog.exec() == QDialog::Accepted) compItem->updateFromInstance(inst);
        } else if (inst.rawTypeStr == "PROBE") {
            ProbeEditorDialogQt dialog(inst, this);
            if (dialog.exec() == QDialog::Accepted) compItem->updateFromInstance(inst);
        } else if (inst.rawTypeStr == "PWM_MASTER") {
            PwmMasterDialogQt dialog(inst, this);
            if (dialog.exec() == QDialog::Accepted) compItem->updateFromInstance(inst);
        }
    }

    QGraphicsView::mouseDoubleClickEvent(event);
}

void SchematicView::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && m_isWiring) {
        m_isWiring = false;
        m_tempWireItem->setVisible(false);
        return;
    }

    if (event->modifiers() & Qt::ControlModifier) {
        if (event->key() == Qt::Key_C) { copySelected(); return; }
        if (event->key() == Qt::Key_V) { pasteSelected(); return; }
        if (event->key() == Qt::Key_D) { duplicateSelected(); return; }
    }

    if (event->key() == Qt::Key_R) {
        for (QGraphicsItem* item : m_scene->selectedItems()) {
            if (ComponentItem* compItem = dynamic_cast<ComponentItem*>(item)) {
                ComponentInstance inst = compItem->getInstance();
                inst.rotation = (inst.rotation + 90) % 360;
                compItem->updateFromInstance(inst);
                updateWires();
            }
        }
    }

    if (event->key() == Qt::Key_H) { flipHorizontal(); return; }
    if (event->key() == Qt::Key_V) { flipVertical(); return; }

    if (event->key() == Qt::Key_Delete) {
        for (QGraphicsItem* item : m_scene->selectedItems()) {
            if (ComponentItem* compItem = dynamic_cast<ComponentItem*>(item)) {
                std::string targetId = compItem->getInstance().id;
                m_design.wires.erase(
                    std::remove_if(m_design.wires.begin(), m_design.wires.end(),
                                   [&targetId](const WireInstance& w) { return w.from.compId == targetId || w.to.compId == targetId; }),
                    m_design.wires.end()
                );
                m_scene->removeItem(compItem);
                delete compItem;
                updateWires();
            }
        }
    }

    QGraphicsView::keyPressEvent(event);
}

} // namespace CircuitSim
