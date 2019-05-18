#include "QtOSGScene.h"
#include "Manipulator.h"
#include "OSGScanModel.h"
#include "FileOperaterClass.h"


#include <osg/MatrixTransform>
#include <osgUtil/Optimizer>
#include <osgFX/SpecularHighlights>
#include <osg/Material>
#include <osg/ShapeDrawable>

#include <QApplication>
#include <QDebug>
#include <QVector>
#include <QMessageBox>
#include <QProgressDialog>


#define CLAMP(x,min,max)   ( (x<min) ? min : ( (x>max) ? max : x ) )

QtOSGScene::QtOSGScene(QWidget *parent)
	: QOpenGLWidget(parent)
	, _mGraphicsWindow(new osgViewer::GraphicsWindowEmbedded(this->x(),this->y(),this->width(),this->height()))
	, _mViewer(new osgViewer::Viewer)
	, _mCamera(new osg::Camera)
	, _mScanModel(new OSGScanModel)
	, _mRoot(new osg::Group)
	, _mBackGround(createBackground())
	, m_scale(QApplication::desktop()->devicePixelRatio())
	, _mManipulator(new Manipulator)
{
	//设置相机参数.
	_mCamera->setViewport(0, 0, this->width(), this->height());
	_mCamera->setClearColor(osg::Vec4f(180.0 / 255.0, 180.0 / 255.0, 180.0 / 255.0, 1.0f));
	float aspectRatio = static_cast<float>(this->width()) / static_cast<float>(this->height());
	_mCamera->setProjectionMatrixAsPerspective(30.f, aspectRatio, 1.0f, 1000.f);
	_mCamera->setGraphicsContext(_mGraphicsWindow);
	
	_mScanModel->setParent(this);

	//_mScanModel->setMatrix(osg::Matrix::rotate(0, osg::Vec3d(0, 0, 0)) * osg::Matrix::scale(osg::Vec3(0.4, 0.4, 0.4))*osg::Matrix::translate(osg::Vec3(0, 0, 0)));
	////缩放旋转.
	//osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	//mt->addChild(_mRoot.get());
	//mt->setMatrix(osg::Matrix::rotate(0, osg::Vec3d(0, 1, 0)) * osg::Matrix::scale(osg::Vec3(0.4, 0.4, 0.4))*osg::Matrix::translate(osg::Vec3(0, 0, 0)));


	_mViewer->setCamera(_mCamera);

	//设置旋转球.
	_mManipulator->setAllowThrow(false);

	//设置鼠标跟踪.
	this->setMouseTracking(true);

	//在场景中设置相机机械辅助手
	_mViewer->setCameraManipulator(_mManipulator);

	//单线程使用.
	_mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);


	//添加背景子节点.
	_mRoot->addChild(_mBackGround.get());
	
	//设置模型中的数据.
	_mRoot->addChild(_mScanModel.get());
	

	//优化场景.
	_mViewer->setSceneData(_mRoot.get());
	_mViewer->realize();

	//设置关联槽函数.这里在添加点云的时候，进行相应的视觉上的更新.
	connect(_mScanModel, &OSGScanModel::homeView, this, &QtOSGScene::homeView);
	connect(_mScanModel, &OSGScanModel::updateCameraDirection, this, &QtOSGScene::updateCameraDirection);

}

QtOSGScene::~QtOSGScene()
{

}

void QtOSGScene::addNodeModel(osg::ref_ptr<osg::Node> loadeModel)
{

	_mScanModel->addChild(loadeModel.get());
	_mScanModel->addChild(_mBackGround.get());
	update();
}

void QtOSGScene::addNode(QString filename)
{
	osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;

	QFile file(filename);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		while (!file.atEnd())
		{
			QByteArray line = file.readLine();
			QString str(line);
			str.replace("\n", "");
			QStringList info = str.split(" ");
			v->push_back(osg::Vec3(info[0].toFloat(), info[1].toFloat(), info[2].toFloat()));
			n->push_back(osg::Vec3(info[3].toFloat(), info[4].toFloat(), info[5].toFloat()));
			c->push_back(osg::Vec4(info[6].toFloat() / 255.f, info[7].toFloat() / 255.f, info[8].toFloat() / 255.f, 1.0f));
		}
		file.close();
	}

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	geom->setVertexArray(v.get());
	geom->setColorArray(c.get());
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->setNormalArray(n.get());
	geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v->size()));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());

	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	geode->getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::ON);

	osgUtil::Optimizer optimizer;
	optimizer.optimize(geode.get());

	osg::ref_ptr<osgFX::SpecularHighlights> shl = new osgFX::SpecularHighlights;
	shl->setTextureUnit(0);
	shl->setLightNumber(1);
	shl->setSpecularColor(osg::Vec4(0.3f, 0.3f, 0.3f, 0.6f));
	shl->setSpecularExponent(16.0);
	shl->addChild(geode.get());

	_mScanModel->addChild(shl.get());
	_mScanModel->addChild(_mBackGround.get());
	update();
}

void QtOSGScene::addNode(QString filename, bool isStruc)
{
	FileOperaterClass FileReadObj;
	vector<AscDataD3Struct> AscDataInfos;

	//将Qstring
	filename.replace("/","\\");
	std::string ascFilestr = std::string((const char*)filename.toLocal8Bit());

	FileReadObj.ReadAscFile(ascFilestr, AscDataInfos);
	

	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array();
	osg::ref_ptr<osg::Vec3Array> coordNormal = new osg::Vec3Array();

	int lengNum = AscDataInfos.size();
	for (int i = 0; i < AscDataInfos.size(); i++)
	{
		coords->push_back(osg::Vec3(AscDataInfos[i].x/8, AscDataInfos[i].y/8, AscDataInfos[i].z/8));
		color->push_back(osg::Vec4(AscDataInfos[i].R / 255.0f, AscDataInfos[i].G/ 255.0f, AscDataInfos[i].B/255.0f, 0));
		coordNormal->push_back(osg::Vec3(AscDataInfos[i].normalX, AscDataInfos[i].normalY, AscDataInfos[i].normalZ));
	}


	//创建几何体
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
	//设置顶点数组
	geometry->setVertexArray(coords.get());
	geometry->setColorArray(color.get());
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->setNormalArray(coordNormal.get());
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coords.get()->size())); //设置关联方式




	//添加到叶节点
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geometry.get());


	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	geode->getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::ON);

	osgUtil::Optimizer optimizer;
	optimizer.optimize(geode.get());

	osg::ref_ptr<osgFX::SpecularHighlights> shl = new osgFX::SpecularHighlights;
	shl->setTextureUnit(0);
	shl->setLightNumber(1);
	shl->setSpecularColor(osg::Vec4(0.3f, 0.3f, 0.3f, 0.6f));
	shl->setSpecularExponent(16.0);
	
	shl->addChild(geode.get());

	_mScanModel->addChild(shl.get());
	_mScanModel->addChild(_mBackGround.get());
	update();
}

void QtOSGScene::AddPointCloudNode(AscDataD3Struct * vector, int Size)
{
	_mScanModel->AddPointCloudNode(vector, Size);
	update();
}

void QtOSGScene::saveCloud(QString s)
{
	if (!_mScanModel->saveCloud(s))
	{
		QMessageBox::warning(this, "save waring", "cloud is empty");
	}
}

void QtOSGScene::clearScene()
{
	_mScanModel->clearNode();
	update();
}

void QtOSGScene::setSceneDirection(CameraView view)
{
	_mManipulator->setCameraView(view);
	update();
}

void QtOSGScene::initRectMove(bool active)
{

}

void QtOSGScene::updateCameraDirection(osg::Vec3 center, osg::Vec3 normal)
{

	double viewDistance = 120;
	osg::Vec3d up(0.0, 1.0, 0.0);
	osg::Vec3d viewDirection(0.0, -1.0, 0.5);
	viewDirection = normal;
	osg::Vec3d eye = center + viewDirection * viewDistance;
	_mViewer->getCameraManipulator()->setHomePosition(eye, center, up);
	_mViewer->home();

	osg::Camera*camera = _mViewer->getCamera();
	osg::Matrix VPW = camera->getViewMatrix() *camera->getProjectionMatrix()
		*camera->getViewport()->computeWindowMatrix();
	osg::Vec3 window = center * VPW;

	int zoom = 30;

	//这个设置框的朝向.
	/*_rectMoveWidget->updateRect(QPoint(window.x() + zoom, window.y() + zoom * 1.5),
		QPoint(window.x() - zoom, window.y() - zoom * 1.5));*/

	update();

}

void QtOSGScene::scanStopped()
{
	//_rectMoveWidget->scanStopped();

}


void QtOSGScene::paintGL()
{
	_mViewer->frame();
}

void QtOSGScene::resizeGL(int width, int height)
{
	this->getEventQueue()->windowResize(this->x()*m_scale, this->y() * m_scale, width*m_scale, height*m_scale);
	_mGraphicsWindow->resized(this->x()*m_scale, this->y() * m_scale, width*m_scale, height*m_scale);
	osg::Camera* camera = _mViewer->getCamera();
	camera->setViewport(0, 0, this->width()*m_scale, this->height()* m_scale);
}

void QtOSGScene::mouseMoveEvent(QMouseEvent * event)
{
	this->getEventQueue()->mouseMotion(event->x()*m_scale, event->y()*m_scale);
}

void QtOSGScene::mousePressEvent(QMouseEvent * event)
{
	unsigned int button = 0;
	switch (event->button())
	{
	case Qt::LeftButton:
		button = 1;
		break;
	case Qt::MiddleButton:
		button = 2;
		break;
	case Qt::RightButton:
		button = 3;
		break;
	default:
		break;
	}
	this->getEventQueue()->mouseButtonPress(event->x()*m_scale, event->y()*m_scale, button);
}

void QtOSGScene::mouseReleaseEvent(QMouseEvent * event)
{
	unsigned int button = 0;
	switch (event->button())
	{
	case Qt::LeftButton:
		button = 1;
		break;
	case Qt::MiddleButton:
		button = 2;
		break;
	case Qt::RightButton:
		button = 3;
		break;
	default:
		break;
	}
	this->getEventQueue()->mouseButtonRelease(event->x()*m_scale, event->y()*m_scale, button);
}

void QtOSGScene::wheelEvent(QWheelEvent * event)
{
	int delta = event->delta();
	osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?
		osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
	this->getEventQueue()->mouseScroll(motion);
}

bool QtOSGScene::event(QEvent * event)
{
	if (event == nullptr)
	{
		return false;
	}

	bool handled = QOpenGLWidget::event(event);

	switch (event->type())
	{
	case QEvent::KeyPress:
	case QEvent::KeyRelease:
	case QEvent::MouseButtonDblClick:
	case QEvent::MouseButtonPress:
	case QEvent::MouseButtonRelease:
	case QEvent::MouseMove:
	case QEvent::Wheel:
		this->update();
		break;
	default:
		break;
	}

	return handled;
}

osgGA::EventQueue * QtOSGScene::getEventQueue() const
{
	osgGA::EventQueue* eventQueue = _mGraphicsWindow->getEventQueue();
	return eventQueue;
}

osg::Group * QtOSGScene::createBackground()
{
	osg::ref_ptr<osg::Vec4Array> m_bgColors = new osg::Vec4Array;

	osg::ref_ptr<osg::Geode> bgGeode = new osg::Geode;
	bgGeode->setName("Background");

	osg::ref_ptr<osg::Projection> HUDProjectionMatrix = new osg::Projection;

	HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0, 1024, 0, 1024));
	osg::ref_ptr<osg::MatrixTransform> HUDModelViewMatrix = new osg::MatrixTransform;
	HUDModelViewMatrix->setMatrix(osg::Matrix::identity());

	HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	HUDProjectionMatrix->addChild(HUDModelViewMatrix);
	HUDModelViewMatrix->addChild(bgGeode);

	osg::ref_ptr<osg::Geometry> HUDBackgroundGeometry = new osg::Geometry;
	HUDBackgroundGeometry->setUseDisplayList(false);

	osg::ref_ptr<osg::Vec3Array> HUDBackgroundVertices = new osg::Vec3Array;
	HUDBackgroundVertices->push_back(osg::Vec3(0, 0, -100));
	HUDBackgroundVertices->push_back(osg::Vec3(1024, 0, -100));
	HUDBackgroundVertices->push_back(osg::Vec3(1024, 1024, -100));
	HUDBackgroundVertices->push_back(osg::Vec3(0, 1024, -100));

	osg::ref_ptr<osg::DrawElementsUInt> HUDBackgroundIndices = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
	HUDBackgroundIndices->push_back(0);
	HUDBackgroundIndices->push_back(1);
	HUDBackgroundIndices->push_back(2);
	HUDBackgroundIndices->push_back(3);

	setGradientBgColor(m_bgColors.get());

	osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array(4);
	(*texcoords)[0].set(0.0f, 0.0f);
	(*texcoords)[1].set(1.0f, 0.0f);
	(*texcoords)[2].set(1.0f, 1.0f);
	(*texcoords)[3].set(0.0f, 1.0f);

	HUDBackgroundGeometry->setTexCoordArray(0, texcoords);

	osg::ref_ptr<osg::Vec3Array> HUDnormals = new osg::Vec3Array;
	HUDnormals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	HUDBackgroundGeometry->setNormalArray(HUDnormals);
	HUDBackgroundGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	HUDBackgroundGeometry->addPrimitiveSet(HUDBackgroundIndices);
	HUDBackgroundGeometry->setVertexArray(HUDBackgroundVertices);
	HUDBackgroundGeometry->setColorArray(m_bgColors.get());
	HUDBackgroundGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	bgGeode->addDrawable(HUDBackgroundGeometry);

	osg::ref_ptr<osg::StateSet> HUDStateSet = new osg::StateSet;
	bgGeode->setStateSet(HUDStateSet);

	HUDStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	HUDStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	HUDStateSet->setRenderBinDetails(-1, "RenderBin");

	HUDProjectionMatrix->setNodeMask(0x1);
	osg::ref_ptr<osg::Group> background = new osg::Group;
	background->addChild(HUDProjectionMatrix.get());
	return background.release();
}

void QtOSGScene::setGradientBgColor(osg::Vec4Array * colors)
{
	osg::Vec4f m_clearColor = osg::Vec4f(180.0 / 255.0, 180.0 / 255.0, 180.0 / 255.0, 1.0f);
	colors->clear();
	colors->push_back(osg::Vec4f(CLAMP(m_clearColor[0] - 0.1, 0., 1.), CLAMP(m_clearColor[1] - 0.1, 0., 1.), CLAMP(m_clearColor[2] - 0.1, 0., 1.), 1.0f));
	colors->push_back(osg::Vec4f(CLAMP(m_clearColor[0] - 0.1, 0., 1.), CLAMP(m_clearColor[1] - 0.1, 0., 1.), CLAMP(m_clearColor[2] - 0.1, 0., 1.), 1.0f));
	colors->push_back(osg::Vec4f(CLAMP(m_clearColor[0] + 0.2, 0., 1.), CLAMP(m_clearColor[1] + 0.2, 0., 1.), CLAMP(m_clearColor[2] + 0.2, 0., 1.), 1.0f));
	colors->push_back(osg::Vec4f(CLAMP(m_clearColor[0] + 0.2, 0., 1.), CLAMP(m_clearColor[1] + 0.2, 0., 1.), CLAMP(m_clearColor[2] + 0.2, 0., 1.), 1.0f));

}
void QtOSGScene::homeView()
{

	double viewDistance = 120;

	osg::Vec3d up(0.0, -1.0, 0.0);
	osg::Vec3d viewDirection(0.0, -1.0, 0.5);

	osg::Vec3d center(0, 0, 0);
	osg::Vec3d eye = center + viewDirection * viewDistance;

	_mViewer->getCameraManipulator()->setHomePosition(eye, center, up);
	_mViewer->home();
	_mManipulator->setCameraView(CameraView::BOTTOM);
	update();
}