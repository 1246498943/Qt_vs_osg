#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_osgRender.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QButtonGroup>
#include <QDebug>

class QtOSGScene;
class osgRender : public QMainWindow
{
	Q_OBJECT

public:

	osgRender(QWidget *parent = Q_NULLPTR);

private slots:

        void onAddAscFileClicked(bool isChecked);

        void onAddOstrFileClicked(bool isChecked);

        void onAddModelClicked(bool isChecked);

		void onAddDrawAble(bool isChecked);

private:

	Ui::osgRenderClass ui;

};
