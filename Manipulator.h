#pragma once
#include <osg/ref_ptr>
#include <osg/observer_ptr>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgGA/TrackballManipulator>
#include "Utils.h"

//轨迹球
class Manipulator :public osgGA::TrackballManipulator
{

public:
	
	//鼠标左中右三键操作.
	virtual bool performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy);

	virtual bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy);

	virtual bool performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy);

	//设置视角.
	void setCameraView(CameraView style);

private:

	//包裹旋转.
	virtual bool wrapRotation(const double eventTimeDelta, const double dx, const double dy);

	//包裹
	virtual bool wrapPan(const double eventTimeDelta, const double dx, const double dy);

};

