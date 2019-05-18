#pragma once

#include <osg/MatrixTransform>
#include <QObject>
#include "Utils.h"
class QString;
class QWidget;

class OSGScanModel :public QObject, public  osg::MatrixTransform
{
	Q_OBJECT

public:

	OSGScanModel();

	~OSGScanModel();

	void setParent(QWidget*widget);

	void AddPointCloudNode(AscDataD3Struct * vector, int Size);

	bool saveCloud(QString s);

	void clearNode();

signals:

	void homeView();

	void updateCameraDirection(osg::Vec3 center, osg::Vec3 normal);

private:

	QWidget*parentWidget;

	bool isNullData(AscDataD3Struct &vector);
};
