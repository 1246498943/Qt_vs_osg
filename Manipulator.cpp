#include "Manipulator.h"


bool Manipulator::performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	return false;
}

bool Manipulator::performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	return this->wrapPan(eventTimeDelta, dx, dy);
}

bool Manipulator::performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	return this->wrapRotation(eventTimeDelta, dx, dy);
}

void Manipulator::setCameraView(CameraView style)
{
	switch (style)
	{
	case CameraView::LEFT:
	{
		//旋转角度，四元素，xyz 三个轴旋转多少度.
		this->setRotation(osg::Quat(osg::inDegrees(0.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(270.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	case CameraView::RIGHT:
	{
		this->setRotation(osg::Quat(osg::inDegrees(0.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(90.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	case CameraView::BOTTOM:
	{
		this->setRotation(osg::Quat(osg::inDegrees(0.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(180.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	case CameraView::TOP:
	{
		this->setRotation(osg::Quat(osg::inDegrees(0.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	case CameraView::FRONT:
	{
		this->setRotation(osg::Quat(osg::inDegrees(90.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	case CameraView::BACK:
	{
		this->setRotation(osg::Quat(osg::inDegrees(90.0f), osg::Vec3(1.0f, 0.0f, 0.0f),
			osg::inDegrees(0.0f), osg::Vec3(0.0f, 1.0f, 0.0f),
			osg::inDegrees(180.0f), osg::Vec3(0.0f, 0.0f, 1.0f)));
		break;
	}
	}

}

bool Manipulator::wrapRotation(const double eventTimeDelta, const double dx, const double dy)
{
	//得到垂直轴固定.
	if (getVerticalAxisFixed())
		rotateWithFixedVertical(dx, dy);
	else
		rotateTrackball(_ga_t0->getXnormalized(), _ga_t0->getYnormalized(),
			_ga_t1->getXnormalized(), _ga_t1->getYnormalized(),
			getThrowScale(eventTimeDelta));
	return true;
}

bool Manipulator::wrapPan(const double eventTimeDelta, const double dx, const double dy)
{
	float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
	panModel(dx*scale, dy*scale);
	return true;
}
