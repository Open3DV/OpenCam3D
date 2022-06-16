#include "camera_gui.h"
#include <QtWidgets/QApplication>
#include <QTextCodec>

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);

	camera_gui w;
	w.show();
	return a.exec();
}
