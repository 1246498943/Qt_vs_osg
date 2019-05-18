#include "osgRender.h"
#include "QtOSGScene.h"
#include "CommonFunctions.h"

osgRender::osgRender(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	connect(ui.actionadd_asc_file,&QAction::triggered,this,&osgRender::onAddAscFileClicked);
	connect(ui.actionadd_osgt_file, &QAction::triggered, this, &osgRender::onAddOstrFileClicked);
	connect(ui.actionadd_model_file, &QAction::triggered, this, &osgRender::onAddModelClicked);
	connect(ui.actionadd_DrawAble_model,&QAction::triggered,this,&osgRender::onAddDrawAble);
}


void osgRender::onAddAscFileClicked(bool isChecked)
{
	QString FileTypes = QString("asc file(*.asc)");
	QString readName = QFileDialog::getOpenFileName(this,tr("Select File"),QDir::currentPath(),FileTypes);
	if (readName.isEmpty())
	{
		return;
	}
	else
	{
		QFileInfo fi(readName);
		if (!fi.exists())
		{
			QMessageBox::warning(this,tr("Failed to Open"), QString("File '%1' not exist").arg(readName));
			return;
		}
		else
		{
			ui.sceneview->clearScene();
			ui.sceneview->addNode(readName,true);
		}
	}


}

void osgRender::onAddOstrFileClicked(bool isChecked)
{
	QString FileTypes = QString("asc file(*.osgt)");
	QString readName = QFileDialog::getOpenFileName(this, tr("Select File"), QDir::currentPath(), FileTypes);
	if (readName.isEmpty())
	{
		return;
	}
	else
	{
		QFileInfo fi(readName);
		if (!fi.exists())
		{
			QMessageBox::warning(this, tr("Failed to Open"), QString("File '%1' not exist").arg(readName));
			return;
		}
		else
		{
			readName.replace("/","\\");
			const std::string strPath = std::string((const char*)readName.toLocal8Bit());

			osg::ref_ptr<osg::Node>  loadedModel = osgDB::readNodeFile(strPath);
		
			ui.sceneview->addNodeModel(loadedModel);
		}
	}

	
}

void osgRender::onAddModelClicked(bool isChecked)
{

	ui.sceneview->clearScene();
	
	osg::ref_ptr<osg::Geode> geod = CommonFunctions::createSphere(osg::Vec3f(1,1,1),osg::Vec4d(155,155,45,1),20);

	//添加自己创建的模型.
	ui.sceneview->addNodeModel(geod);
}

void osgRender::onAddDrawAble(bool isChecked)
{

}


