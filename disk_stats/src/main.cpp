#include <QApplication>

#include "mainwindow.hpp"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	view::MainWindow window;

	return app.exec();
}