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


//通用的方法类.
class CommonFunctions
{
public:
	
	CommonFunctions() {};

	virtual ~CommonFunctions() {};

	//两点之间的距离.
	static double DistanceBetweenPoints(osg::Vec3d pt1, osg::Vec3d pt2);

	//三角形的重心位置.实际计算的时候，就是在计算中心的时候，乘以了权重.
	static osg::Vec3d GravityPositionOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3);

	//法向归一化.
	static osg::Vec3d NormalVectorOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3);

	//四元数的欧拉角.
	static osg::Vec3d QuaternionToEulerAngles(osg::Quat q);

	//设置材质.
	static void setSameGLFaceMaterial(osg::StateSet* state, osg::Vec4d color);

	//清理面片的材质.
	static void setClearGLFaceMaterial(osg::StateSet* state, osg::Vec4d color);

	//设置面pain的材质.
	static void setGLFaceMaterial(osg::StateSet* state);

	//矢量的最大数值.
	static double maxmiumValueOfVector(osg::Vec3d v);

	//创建小球.
	static osg::Geode* createSphere(osg::Vec3d point, double r, osg::Vec4d color);

	//计算点的投影.
	static osg::Vec3d calProjectionOfPoint(osg::Vec4d plane, osg::Vec3d point);

	//计算xz方向上的平移矩阵.
	static osg::Matrixd calXOZPlaneTransMat(osg::Vec3d pZ);

	//创建asix
	static osg::Geode* createAsix(osg::Vec4 color, QString id);

	//创建plane
	static osg::Geode* createPlane();

	//创建小球.
	static osg::Geode* createSphere(osg::Vec3d center, osg::Vec4d color, float r);

	//创建一个几何圆
	static osg::Geometry* createCircleGeometry(float radius, unsigned int numSegments);

	//创建文字对象.
	static osgText::Text* createTextObject(osg::Vec3d position, QString s, bool border);

	//向量和面之间的角度.
	static double AngleBetweenVecsWithoutABS(osg::Vec3d pt1, osg::Vec3d pt2);

	//两个向量之间的角度.
	static double AngleBetweenVecs(osg::Vec3d pt1, osg::Vec3d pt2);

	//获取中文字
	static QString getChineseString(QString s);

	//获取当前投影模型的点.
	static osg::Vec3d getProjectPoint(osg::Vec4d plane, osg::Vec3d point);

	//加密字符串.
	static QString EncryptString(QString s);

	//解密字符串.
	static QString DecryptString(QString s);

	//获取汉字的.firstLetters
	static QString getFirstLettersOfChinese(QString src);

};

#endif // ! COMMONFUNCTIONS_H
