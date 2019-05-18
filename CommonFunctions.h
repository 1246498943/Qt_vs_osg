#pragma once
#ifndef  COMMONFUNCTIONS_H
#define  COMMONFUNCTIONS_H
#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>
#include <osgText/Text>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <QVector>
#include <QString>
#include <QDebug>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/BlendColor>
#include <osg/BlendFunc>
#include <osg/LineWidth>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgDB/ReadFile>

#include "Utils.h"


//ͨ�õķ�����.
class CommonFunctions
{
public:
	
	CommonFunctions() {};

	virtual ~CommonFunctions() {};

	//����֮��ľ���.
	static double DistanceBetweenPoints(osg::Vec3d pt1, osg::Vec3d pt2);

	//�����ε�����λ��.ʵ�ʼ����ʱ�򣬾����ڼ������ĵ�ʱ�򣬳�����Ȩ��.
	static osg::Vec3d GravityPositionOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3);

	//�����һ��.
	static osg::Vec3d NormalVectorOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3);

	//��Ԫ����ŷ����.
	static osg::Vec3d QuaternionToEulerAngles(osg::Quat q);

	//���ò���.
	static void setSameGLFaceMaterial(osg::StateSet* state, osg::Vec4d color);

	//������Ƭ�Ĳ���.
	static void setClearGLFaceMaterial(osg::StateSet* state, osg::Vec4d color);

	//������pain�Ĳ���.
	static void setGLFaceMaterial(osg::StateSet* state);

	//ʸ���������ֵ.
	static double maxmiumValueOfVector(osg::Vec3d v);

	//����С��.
	static osg::Geode* createSphere(osg::Vec3d point, double r, osg::Vec4d color);

	//������ͶӰ.
	static osg::Vec3d calProjectionOfPoint(osg::Vec4d plane, osg::Vec3d point);

	//����xz�����ϵ�ƽ�ƾ���.
	static osg::Matrixd calXOZPlaneTransMat(osg::Vec3d pZ);

	//����asix
	static osg::Geode* createAsix(osg::Vec4 color, QString id);

	//����plane
	static osg::Geode* createPlane();

	//����С��.
	static osg::Geode* createSphere(osg::Vec3d center, osg::Vec4d color, float r);

	//����һ������Բ
	static osg::Geometry* createCircleGeometry(float radius, unsigned int numSegments);

	//�������ֶ���.
	static osgText::Text* createTextObject(osg::Vec3d position, QString s, bool border);

	//��������֮��ĽǶ�.
	static double AngleBetweenVecsWithoutABS(osg::Vec3d pt1, osg::Vec3d pt2);

	//��������֮��ĽǶ�.
	static double AngleBetweenVecs(osg::Vec3d pt1, osg::Vec3d pt2);

	//��ȡ������
	static QString getChineseString(QString s);

	//��ȡ��ǰͶӰģ�͵ĵ�.
	static osg::Vec3d getProjectPoint(osg::Vec4d plane, osg::Vec3d point);

	//�����ַ���.
	static QString EncryptString(QString s);

	//�����ַ���.
	static QString DecryptString(QString s);

	//��ȡ���ֵ�.firstLetters
	static QString getFirstLettersOfChinese(QString src);

};

#endif // ! COMMONFUNCTIONS_H
