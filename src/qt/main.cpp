#include <QApplication>
#include "MainWindow.hpp"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("CircuitSim Pro");
    app.setOrganizationName("CircuitSim");

    CircuitSim::MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
