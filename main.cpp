#include "osgRender.h"
#include <QtWidgets/QApplication>
#include <QTextStream>
#include <QTranslator>
#include <QtPlugin>
#include <osg/Vec3>
#include <QsurFace>

int main(int argc, char *argv[])
{
	QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

	qRegisterMetaType<osg::Vec3>("osg::Vec3");

	QApplication a(argc, argv);


	//定义了OPENGL的版本信息.
	QSurfaceFormat format;
	format.setVersion(2, 1);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	QSurfaceFormat::setDefaultFormat(format);
	
	osgRender w;
	w.show();



	return a.exec();
}
