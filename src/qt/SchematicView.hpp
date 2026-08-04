#pragma once

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include "ComponentItem.hpp"
#include "WireItem.hpp"
#include "QtDialogs.hpp"
#include "../engine/Components.hpp"

namespace CircuitSim {

class SchematicView : public QGraphicsView {
    Q_OBJECT
public:
    explicit SchematicView(QWidget* parent = nullptr);

    void setCircuit(const CircuitDesign& design);
    CircuitDesign getCircuit() const;
    void addComponent(const ComponentInstance& comp);

    void copySelected();
    void pasteSelected();
    void duplicateSelected();
    void flipHorizontal();
    void flipVertical();
    void exportSVG(const QString& filePath);

signals:
    void componentSelected(const ComponentInstance* comp);

protected:
    void drawBackground(QPainter* painter, const QRectF& rect) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    QGraphicsScene* m_scene;
    CircuitDesign m_design;
    
    // Wiring state
    bool m_isWiring = false;
    QString m_wireStartCompId;
    QString m_wireStartPin;
    QGraphicsPathItem* m_tempWireItem = nullptr;
    
    // Pan state
    bool m_isPanning = false;
    QPoint m_lastPanPos;

    // Clipboard cache
    QString m_clipboardJson;

    void updateWires();
};

} // namespace CircuitSim
