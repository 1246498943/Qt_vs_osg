#include "OSGScanModel.h"
#include <QDebug>
#include <QWidget>
#include <QProgressDialog>
#include <QApplication>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Material>
#include <osgUtil/Optimizer>


OSGScanModel::OSGScanModel()
	
{
}

OSGScanModel::~OSGScanModel()
{
}

void OSGScanModel::setParent(QWidget * widget)
{
	parentWidget = widget;
}

void OSGScanModel::AddPointCloudNode(AscDataD3Struct * vector, int Size)
{
	osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;

	osg::Vec3 normalEstimate(0, 0, 0);
	osg::Vec3 centerEstimate(0, 0, 0);
	int estimateCount = 0;
	int sampleCount = Size * 0.95;

	for (int i = 0; i < Size; i++)
	{
		osg::Vec3 vertex(vector[i].x, vector[i].y, vector[i].z);
		osg::Vec3 normal(vector[i].normalX, vector[i].normalY, vector[i].normalZ);
		osg::Vec4 color((int)vector[i].R / 255.f, (int)vector[i].G / 255.f, (int)vector[i].B / 255.f, 1.0f);

		v->push_back(vertex);
		n->push_back(normal);
		c->push_back(color);

		if (i > sampleCount)
		{
			normalEstimate += normal;
			centerEstimate += vertex;
			estimateCount++;
		}
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

	if (this->getNumChildren() > 0)
	{
		this->replaceChild(this->getChild(0), geode.get());

		centerEstimate /= estimateCount;
		normalEstimate = normalEstimate * this->getMatrix();
		normalEstimate.normalize();
		centerEstimate = centerEstimate * this->getMatrix();
		emit updateCameraDirection(centerEstimate, normalEstimate);
		//qDebug() << "geode replaced";
	}
	else
	{
		this->addChild(geode.get());
		osg::Vec3 center = geode->getBound().center();
		this->setMatrix(osg::Matrix::inverse(osg::Matrix::translate(center))
			*osg::Matrix::rotate(osg::inDegrees(90.f), osg::Vec3(0, 0, 1)));
		emit homeView();
	}

}

bool OSGScanModel::saveCloud(QString s)
{

	return true;
	/*if (this->getNumChildren() > 0)
	{
		osg::Geometry*geom = this->getChild(0)->asGeode()->getChild(0)->asGeometry();
		osg::Vec3Array*vertexs = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		QProgressDialog dialog("save progress", "", 0, vertexs->size(), parentWidget);
		dialog.setCancelButton(0);
		dialog.setWindowTitle("saving");
		dialog.show();
		dialog.setValue(0);

		MyMesh cloudmesh;
		for (int i = 0; i < vertexs->size(); i++)
		{
			osg::Vec3 p = vertexs->at(i);
			cloudmesh.add_vertex(MyMesh::Point(p[0], p[1], p[2]));
			dialog.setValue(dialog.value() + 1);
			QApplication::processEvents();
		}
		return OpenMesh::IO::write_mesh(cloudmesh, std::string(s.toLocal8Bit()));
	}
	return false;*/
}

void OSGScanModel::clearNode()
{
	if (this->getNumChildren() > 0)
	{
		this->removeChildren(0, this->getNumChildren());
	}
}
