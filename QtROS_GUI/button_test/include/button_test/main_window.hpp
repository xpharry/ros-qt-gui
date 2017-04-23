/**
 * @file /include/button_test/main_window.hpp
 *
 * @brief Qt based gui for button_test.
 *
 * @date November 2010
 **/
#ifndef button_test_MAIN_WINDOW_H
#define button_test_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QStringListModel>
#include <QThread>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace button_test {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
	class MainWindow : public QMainWindow {
		Q_OBJECT

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void showButtonTestMessage();

	public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check);
	void on_button_test_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void moveLeft();
//    void moveRight();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QStringListModel* logging_model;
};

}  // namespace button_test

#endif // button_test_MAIN_WINDOW_H
