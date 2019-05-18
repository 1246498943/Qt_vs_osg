/********************************************************************************
** Form generated from reading UI file 'osgRender.ui'
**
** Created by: Qt User Interface Compiler version 5.9.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OSGRENDER_H
#define UI_OSGRENDER_H

#include <QtCore/QVariant>
#include <QtOSGScene.h>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_osgRenderClass
{
public:
    QAction *actionadd_asc_file;
    QAction *actionadd_osgt_file;
    QAction *actionadd_model_file;
    QAction *actionadd_DrawAble_model;
    QAction *action_add_Drawable;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QtOSGScene *sceneview;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *osgRenderClass)
    {
        if (osgRenderClass->objectName().isEmpty())
            osgRenderClass->setObjectName(QStringLiteral("osgRenderClass"));
        osgRenderClass->resize(680, 576);
        actionadd_asc_file = new QAction(osgRenderClass);
        actionadd_asc_file->setObjectName(QStringLiteral("actionadd_asc_file"));
        actionadd_osgt_file = new QAction(osgRenderClass);
        actionadd_osgt_file->setObjectName(QStringLiteral("actionadd_osgt_file"));
        actionadd_model_file = new QAction(osgRenderClass);
        actionadd_model_file->setObjectName(QStringLiteral("actionadd_model_file"));
        actionadd_DrawAble_model = new QAction(osgRenderClass);
        actionadd_DrawAble_model->setObjectName(QStringLiteral("actionadd_DrawAble_model"));
        action_add_Drawable = new QAction(osgRenderClass);
        action_add_Drawable->setObjectName(QStringLiteral("action_add_Drawable"));
        centralWidget = new QWidget(osgRenderClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        sceneview = new QtOSGScene(centralWidget);
        sceneview->setObjectName(QStringLiteral("sceneview"));
        sceneview->setStyleSheet(QLatin1String("QWidget{\n"
"background-color:rgb(185,185,185);\n"
"\n"
"}"));

        gridLayout->addWidget(sceneview, 0, 0, 1, 1);

        osgRenderClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(osgRenderClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 680, 26));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        osgRenderClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(osgRenderClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        osgRenderClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(osgRenderClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        osgRenderClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menu->addSeparator();
        menu->addAction(actionadd_asc_file);
        menu->addSeparator();
        menu->addAction(actionadd_osgt_file);
        menu->addSeparator();
        menu->addAction(actionadd_model_file);
        menu->addSeparator();
        menu->addAction(action_add_Drawable);

        retranslateUi(osgRenderClass);

        QMetaObject::connectSlotsByName(osgRenderClass);
    } // setupUi

    void retranslateUi(QMainWindow *osgRenderClass)
    {
        osgRenderClass->setWindowTitle(QApplication::translate("osgRenderClass", "osgRender", Q_NULLPTR));
        actionadd_asc_file->setText(QApplication::translate("osgRenderClass", "add .asc file", Q_NULLPTR));
        actionadd_osgt_file->setText(QApplication::translate("osgRenderClass", "add .osgt file", Q_NULLPTR));
        actionadd_model_file->setText(QApplication::translate("osgRenderClass", "add.model file", Q_NULLPTR));
        actionadd_DrawAble_model->setText(QApplication::translate("osgRenderClass", "add DrawAble model", Q_NULLPTR));
        action_add_Drawable->setText(QApplication::translate("osgRenderClass", "_add.Drawable", Q_NULLPTR));
        menu->setTitle(QApplication::translate("osgRenderClass", "\346\267\273\345\212\240\346\250\241\345\236\213", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class osgRenderClass: public Ui_osgRenderClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OSGRENDER_H
