/**
 * @file /qlistener/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/Qt_ROS1/main_window.hpp"
#include "../include/Qt_ROS1/listener.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    Listener listener(argc,argv);
    MainWindow w(&listener);
    w.show();
    QObject::connect(&listener,SIGNAL(Update_Image(const QPixmap*)),&w,SLOT(updatePixmap(const QPixmap*)));
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
