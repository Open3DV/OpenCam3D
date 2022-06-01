#include "camera_gui.h"
#include <QtWidgets/QApplication>
#include <QTextCodec>


camera_gui* p_w;

//网格掉线
int main_on_dropped(void* param)
{
	std::cout << "Main Network dropped!" << std::endl;
	p_w->handle_network_drop();
	return 0;
}


int main(int argc, char* argv[])
{
	QApplication a(argc, argv);



	camera_gui w;
	p_w = &w;
	w.setOnDrop(main_on_dropped);
	w.show();



	return a.exec();
}
