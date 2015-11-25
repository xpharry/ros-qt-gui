#include <QApplication>
#include <QLabel>
#include <QWidget>
int main(int argc, char *argv[ ])
{
	QApplication app(argc, argv);
	QLabel hello("<center>Welcome to my first WikiHow Qt program</center>");
	hello.setWindowTitle("My First WikiHow Qt Program");
	hello.resize(400, 400);
	hello.show();
	return app.exec();
}