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

//����OSG����
class Manipulator;
class OSGScanModel;
class FileOperaterClass;


class QtOSGScene : public QOpenGLWidget
{
	Q_OBJECT

public:
	
	QtOSGScene(QWidget *parent);
	
	virtual ~QtOSGScene();

	//���ģ��.
	void addNodeModel(osg::ref_ptr<osg::Node> loadeModel);

	//��ӽڵ�.asc�ļ�.
	void addNode(QString filename);

	//ʹ���ڲ����ȡ�ڵ�.
	void addNode(QString filename,bool isStruc);
	
	//��ӵ���.
	void AddPointCloudNode(AscDataD3Struct * vector, int Size);
	
	//�������.
	void saveCloud(QString s);
	
	//�������.
	void clearScene();
	
	//���ó�������.
	void setSceneDirection(CameraView view);
	
	//��ʼ��һ������.
	void initRectMove(bool active);

public slots:

	//����.
	void homeView();

	//���µ��Ʒ���.
	void updateCameraDirection(osg::Vec3 center, osg::Vec3 normal);
	
	//ֹͣ.
	void scanStopped();

protected:

	virtual void paintGL();//����OpenGL����
	virtual void resizeGL(int width, int height);//��ʼ��OpenGL������osg������λ��
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void wheelEvent(QWheelEvent* event);
	virtual bool event(QEvent* event);

private:

	osgGA::EventQueue* getEventQueue() const;
	osg::Group*createBackground();
	void setGradientBgColor(osg::Vec4Array*colors);
	
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _mGraphicsWindow;//����Ƕ��Windows���ڵ�osgviewer
	osg::ref_ptr<osgViewer::Viewer> _mViewer;
	osg::ref_ptr<osg::Camera> _mCamera;
	osg::ref_ptr<Manipulator>_mManipulator;
	osg::ref_ptr<OSGScanModel> _mScanModel;
	osg::ref_ptr<osg::Group> _mRoot;
	osg::ref_ptr<osg::Group> _mBackGround;
	qreal m_scale;


};
