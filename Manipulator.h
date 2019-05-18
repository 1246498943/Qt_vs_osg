#pragma once
#include <osg/ref_ptr>
#include <osg/observer_ptr>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgGA/TrackballManipulator>
#include "Utils.h"

//�켣��
class Manipulator :public osgGA::TrackballManipulator
{

public:
	
	//�����������������.
	virtual bool performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy);

	virtual bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy);

	virtual bool performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy);

	//�����ӽ�.
	void setCameraView(CameraView style);

private:

	//������ת.
	virtual bool wrapRotation(const double eventTimeDelta, const double dx, const double dy);

	//����
	virtual bool wrapPan(const double eventTimeDelta, const double dx, const double dy);

};

