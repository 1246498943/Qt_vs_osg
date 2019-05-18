#pragma once

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QDesktopWidget>
#include <QString>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventHandler>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>


#include "Utils.h"

//创建OSG场景
class Manipulator;
class OSGScanModel;
class FileOperaterClass;


class QtOSGScene : public QOpenGLWidget
{
	Q_OBJECT

public:
	
	QtOSGScene(QWidget *parent);
	
	virtual ~QtOSGScene();

	//添加模型.
	void addNodeModel(osg::ref_ptr<osg::Node> loadeModel);

	//添加节点.asc文件.
	void addNode(QString filename);

	//使用内部类读取节点.
	void addNode(QString filename,bool isStruc);
	
	//添加点云.
	void AddPointCloudNode(AscDataD3Struct * vector, int Size);
	
	//保存点云.
	void saveCloud(QString s);
	
	//清除场景.
	void clearScene();
	
	//设置场景方向.
	void setSceneDirection(CameraView view);
	
	//初始化一个方框.
	void initRectMove(bool active);

public slots:

	//方向.
	void homeView();

	//更新点云方向.
	void updateCameraDirection(osg::Vec3 center, osg::Vec3 normal);
	
	//停止.
	void scanStopped();

protected:

	virtual void paintGL();//绘制OpenGL场景
	virtual void resizeGL(int width, int height);//初始化OpenGL场景和osg相机相对位置
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void wheelEvent(QWheelEvent* event);
	virtual bool event(QEvent* event);

private:

	osgGA::EventQueue* getEventQueue() const;
	osg::Group*createBackground();
	void setGradientBgColor(osg::Vec4Array*colors);
	
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _mGraphicsWindow;//创建嵌入Windows窗口的osgviewer
	osg::ref_ptr<osgViewer::Viewer> _mViewer;
	osg::ref_ptr<osg::Camera> _mCamera;
	osg::ref_ptr<Manipulator>_mManipulator;
	osg::ref_ptr<OSGScanModel> _mScanModel;
	osg::ref_ptr<osg::Group> _mRoot;
	osg::ref_ptr<osg::Group> _mBackGround;
	qreal m_scale;


};
