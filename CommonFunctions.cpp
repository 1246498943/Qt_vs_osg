#include "CommonFunctions.h"


double CommonFunctions::DistanceBetweenPoints(osg::Vec3d pt1, osg::Vec3d pt2)
{
	double dista = sqrt(pow(pt1.x() - pt2.x(), 2) + pow(pt1.y() - pt2.y(), 2) + pow(pt1.z() - pt2.z(), 2));
	return dista;
}

osg::Vec3d CommonFunctions::GravityPositionOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3)
{
	double o = (pt1.x() + pt2.x() + pt3.x()) / 3;
	double p = (pt1.y() + pt2.y() + pt3.y()) / 3;
	double q = (pt1.z() + pt2.z() + pt3.z()) / 3;
	osg::Vec3d gravityPosition;
	gravityPosition.set(o, p, q);
	return gravityPosition;
}

osg::Vec3d CommonFunctions::NormalVectorOfTriangle(osg::Vec3d pt1, osg::Vec3d pt2, osg::Vec3d pt3)
{
	double d = pt1.y()*pt2.z() + pt2.y()*pt3.z() + pt3.y()*pt1.z() - pt1.y()*pt3.z() - pt2.y()*pt1.z() - pt3.y()*pt2.z();
	double e = pt1.x()*pt2.z() + pt2.x()*pt3.z() + pt3.x()*pt1.z() - pt3.x()*pt2.z() - pt2.x()*pt1.z() - pt1.x()*pt3.z();
	double f = pt1.x()*pt2.y() + pt2.x()*pt3.y() + pt3.x()*pt1.y() - pt1.x()*pt3.y() - pt2.x()*pt1.y() - pt3.x()*pt2.y();
	double g = sqrt(d*d + e * e + f * f);

	double o = d / g;
	double p = e / g;
	double q = f / g;

	osg::Vec3d normalVector;
	normalVector.set(o, p, q);
	return normalVector;
}

osg::Vec3d CommonFunctions::QuaternionToEulerAngles(osg::Quat q)
{
	double heading, pitch, roll;
	double test = q.y() * q.z() + q.x() * q.w();
	double sqx = q.x() * q.x();
	double sqy = q.y() * q.y();
	double sqz = q.z() * q.z();
	heading = atan2(2.0 * q.z() * q.w() - 2.0 * q.y() * q.x(), 1.0 - 2.0 * sqz - 2.0 * sqx);
	pitch = asin(2.0 * test);
	roll = atan2(2.0 * q.y() * q.w() - 2.0 * q.z() * q.x(), 1.0 - 2.0 * sqy - 2.0 * sqx);

	if (test > 0.4999)
	{
		heading = 2.0 * atan2(q.y(), q.w());
		pitch = osg::PI_2;
		roll = 0.0;
	}
	if (test < -0.4999)
	{
		heading = 2.0 * atan2(q.y(), q.w());
		pitch = -osg::PI_2;
		roll = 0.0;
	}

	osg::Vec3d eulerAngles;
	eulerAngles.set(roll, pitch, heading);
	return eulerAngles;
}

void CommonFunctions::setSameGLFaceMaterial(osg::StateSet* state, osg::Vec4d color)
{
	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setDiffuse(osg::Material::FRONT, color);
	material->setAmbient(osg::Material::FRONT, color);
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	state->setAttributeAndModes(material, osg::StateAttribute::ON);
	state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
}

void CommonFunctions::setClearGLFaceMaterial(osg::StateSet* state, osg::Vec4d color)
{
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	state->setMode(GL_BLEND, osg::StateAttribute::ON);
	state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::BlendColor> bc = new osg::BlendColor(color);
	osg::ref_ptr<osg::BlendFunc>bf = new osg::BlendFunc();
	state->setAttributeAndModes(bf, osg::StateAttribute::ON);
	state->setAttributeAndModes(bc, osg::StateAttribute::ON);
	bf->setSource(osg::BlendFunc::CONSTANT_ALPHA);
	bf->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	bc->setConstantColor(color);

	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setDiffuse(osg::Material::FRONT, color);
	material->setAmbient(osg::Material::FRONT, color);
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	state->setAttributeAndModes(material, osg::StateAttribute::ON);
}

void CommonFunctions::setGLFaceMaterial(osg::StateSet* state)
{
	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	state->setAttributeAndModes(material, osg::StateAttribute::ON);
	state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
}

double CommonFunctions::maxmiumValueOfVector(osg::Vec3d v)
{
	double a = fabs(v.x());
	double b = fabs(v.y());
	double c = fabs(v.z());

	if (a > b && a > c) return a;
	if (b > a && b > c) return b;
	return c;
}
osg::Geode* CommonFunctions::createSphere(osg::Vec3d point, double r, osg::Vec4d color)
{
	osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(point, r);
	osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sphere.get());
	sd->setColor(color);
	setGLFaceMaterial(sd->getOrCreateStateSet());
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(sd.get());
	return geode.release();
}

osg::Vec3d CommonFunctions::calProjectionOfPoint(osg::Vec4d plane, osg::Vec3d point)
{
	double dis = (plane.x() * point.x() + plane.y() * point.y() + plane.z() * point.z() + plane.w()) /
		(sqrt(plane.x()*plane.x() + plane.y() * plane.y() + plane.z() * plane.z()));
	double k = -1 * dis / sqrt(plane.x()*plane.x() + plane.y() * plane.y() + plane.z() * plane.z());
	double x = point.x() + k * plane.x();
	double y = point.y() + k * plane.y();
	double z = point.z() + k * plane.z();
	return osg::Vec3d(x, y, z);
}
osg::Matrixd CommonFunctions::calXOZPlaneTransMat(osg::Vec3d pZ)
{
	osg::Vec3d pY = osg::Vec3d(0, -1, 0);
	osg::Vec3d pX;
	pX.set(pY.y()*pZ.z() - pY.z()*pZ.y(), pY.z()*pZ.x() - pY.x()*pZ.z(), pY.x()*pZ.y() - pY.y()*pZ.x());

	pX.normalize();
	pY.normalize();
	pZ.normalize();

	osg::Matrixd trans;

	float pvalue[4][4];
	pvalue[0][0] = pX.x();
	pvalue[0][1] = pX.y();
	pvalue[0][2] = pX.z();
	pvalue[0][3] = 0;
	pvalue[1][0] = pY.x();
	pvalue[1][1] = pY.y();
	pvalue[1][2] = pY.z();
	pvalue[1][3] = 0;
	pvalue[2][0] = pZ.x();
	pvalue[2][1] = pZ.y();
	pvalue[2][2] = pZ.z();
	pvalue[2][3] = 0;
	pvalue[3][0] = 0;
	pvalue[3][1] = 0;
	pvalue[3][2] = 0;
	pvalue[3][3] = 1;

	const float*ptr = pvalue[0];
	trans.set(ptr);

	return trans;
}
osg::Geode* CommonFunctions::createAsix(osg::Vec4 color, QString id)
{
	osg::ref_ptr<osg::Geode> rootGeode = new osg::Geode;

	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(2);
	osg::ref_ptr<osg::ShapeDrawable> shape = nullptr;
	if (id == "cone")
	{
		(*vertices)[0] = osg::Vec3(0.0f, 0.0f, 0.35f);
		(*vertices)[1] = osg::Vec3(0.0f, 0.0f, 1.0f);
		osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(0.0f, 0.0f, 1.0f), 0.10f, 0.50f);
		shape = new osg::ShapeDrawable(cone.get());
	}
	if (id == "box")
	{
		(*vertices)[0] = osg::Vec3(0.0f, 0.0f, 0.0f);
		(*vertices)[1] = osg::Vec3(1.0f, 0.0f, 0.0f);
		osg::ref_ptr<osg::Box> box = new osg::Box(osg::Vec3(1.0f, 0.0f, 0.0f), 0.12);
		shape = new osg::ShapeDrawable(box.get());
	}

	lineGeometry->setVertexArray(vertices.get());
	lineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
	lineGeometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
	linewidth->setWidth(8.0f);
	lineGeometry->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
	rootGeode->addDrawable(lineGeometry.get());

	shape->setColor(color);
	setSameGLFaceMaterial(shape->getOrCreateStateSet(), color);
	rootGeode->addDrawable(shape.get());
	return rootGeode.release();
}
osg::Geode* CommonFunctions::createPlane()
{
	osg::ref_ptr<osg::Geode> root = new osg::Geode;
	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
	(*vertices)[0] = osg::Vec3(0.00f, 0.0f, 0.35f);
	(*vertices)[1] = osg::Vec3(0.00f, 0.0f, 0.00f);
	(*vertices)[2] = osg::Vec3(0.35f, 0.0f, 0.00f);
	(*vertices)[3] = osg::Vec3(0.35f, 0.0f, 0.35f);

	geometry->setVertexArray(vertices.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, vertices->size()));
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0, -1.0, 0.0));
	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

	geometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geometry->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	root->addDrawable(geometry.get());
	return root.release();
}
osg::Geode * CommonFunctions::createSphere(osg::Vec3d center, osg::Vec4d color, float r)
{
	osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
	hints->setDetailRatio(0.5f);
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(center, r);
	osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sphere.get(), hints.get());
	sd->setColor(color);
	geode->addDrawable(sd.get());
	setGLFaceMaterial(geode->getOrCreateStateSet());
	return geode.release();
}
osg::Geometry*  CommonFunctions::createCircleGeometry(float radius, unsigned int numSegments)
{
	const float angleDelta = 2.0f*osg::PI / (float)numSegments;
	const float r = radius;
	float angle = 0.0f;
	osg::ref_ptr<osg::Vec3Array> vertexArray = new osg::Vec3Array(numSegments);
	osg::ref_ptr<osg::Vec3Array> normalArray = new osg::Vec3Array(numSegments);
	for (unsigned int i = 0; i < numSegments; ++i, angle += angleDelta)
	{
		float c = cosf(angle);
		float s = sinf(angle);
		(*vertexArray)[i].set(c*r, s*r, 0.0f);
		(*normalArray)[i].set(c, s, 0.0f);
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray.get());
	geometry->setNormalArray(normalArray.get());
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertexArray->size()));
	return geometry.release();
}

double CommonFunctions::AngleBetweenVecsWithoutABS(osg::Vec3d pt1, osg::Vec3d pt2)
{
	double delta = (pt1.x()*pt2.x() + pt1.y()*pt2.y() + pt1.z()*pt2.z()) /
		sqrt((pt1.x()*pt1.x() + pt1.y()*pt1.y() + pt1.z()*pt1.z())*(pt2.x()*pt2.x() + pt2.y()*pt2.y() + pt2.z()*pt2.z()));
	double theta = acos(delta);
	return theta * 180 / osg::PI;
}

double CommonFunctions::AngleBetweenVecs(osg::Vec3d pt1, osg::Vec3d pt2)
{
	double delta = (pt1.x()*pt2.x() + pt1.y()*pt2.y() + pt1.z()*pt2.z()) /
		sqrt((pt1.x()*pt1.x() + pt1.y()*pt1.y() + pt1.z()*pt1.z())*(pt2.x()*pt2.x() + pt2.y()*pt2.y() + pt2.z()*pt2.z()));
	double theta = acos(fabs(delta));
	return theta * 180 / osg::PI;
}
osgText::Text* CommonFunctions::createTextObject(osg::Vec3d position, QString s, bool border)
{
	osg::ref_ptr<osgText::Text> textObject = new osgText::Text;
	osg::ref_ptr<osgText::Font> arial = osgText::readFontFile("C://Windows//Fonts//arial.ttf");
	textObject->setFont(arial.get());
	textObject->setColor(osg::Vec4(0, 0, 0.6, 1.0));
	textObject->setAlignment(osgText::Text::CENTER_CENTER);//文字显示方向
	textObject->setAxisAlignment(osgText::Text::SCREEN);// 一直朝向镜头
	textObject->setAutoRotateToScreen(true);//跟随视角不断变化
	textObject->setText(s.toStdString());
	textObject->setPosition(position);
	textObject->setCharacterSize(1.0);
	textObject->setFontResolution(32, 32);
	if (border)
	{
		textObject->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);//添加文字边框
	}

	osg::ref_ptr<osg::StateSet> stateset = textObject->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::BlendFunc>bf = new osg::BlendFunc();
	stateset->setAttributeAndModes(bf, osg::StateAttribute::ON);
	return textObject.release();
}

QString CommonFunctions::getChineseString(QString s)
{
	int nCount = s.count();
	QString chs;
	for (int i = 0; i < nCount; i++)
	{
		QChar cha = s.at(i);
		ushort uni = cha.unicode();
		if (uni >= 0x4E00 && uni <= 0x9FA5)
		{
			chs.append(cha);
		}
	}
	return chs;
}

osg::Vec3d CommonFunctions::getProjectPoint(osg::Vec4d plane, osg::Vec3d point)
{
	double dis = (plane.x() * point.x() + plane.y() * point.y() + plane.z() * point.z() + plane.w()) /
		(sqrt(plane.x() * plane.x() + plane.y() * plane.y() + plane.z() * plane.z()));

	double k = -1 * dis / sqrt(plane.x() * plane.x() + plane.y() * plane.y() + plane.z() * plane.z());
	double x = point.x() + k * plane.x();
	double y = point.y() + k * plane.y();
	double z = point.z() + k * plane.z();
	return osg::Vec3d(x, y, z);
}

QString CommonFunctions::EncryptString(QString s)
{
	QString encryptString = s;
	for (int i = 0; i < encryptString.length(); ++i)
	{
		QChar ch = encryptString[i];
		encryptString[i] = QChar::fromLatin1(encryptString[i].toLatin1() - 3);
	}
	QByteArray string = encryptString.toUtf8();
	string = string.toBase64();
	return QString(string);
}

QString CommonFunctions::DecryptString(QString s)
{
	QByteArray string = s.toUtf8();
	string = string.fromBase64(string);
	QString decryptString(string);
	for (int i = 0; i < decryptString.length(); ++i)
	{
		QChar ch = decryptString[i];
		decryptString[i] = QChar::fromLatin1(decryptString[i].toLatin1() + 3);
	}
	return decryptString;
}

QString CommonFunctions::getFirstLettersOfChinese(QString src)
{
	QString firstLetters;
	for (int i = 0; i < src.length(); i++)
	{
		QString str = src.at(i);
		QByteArray arr = str.toLocal8Bit();
		wchar_t wchr;

		if (arr.size() == 1)
		{
			wchr = arr.at(0) & 0xff;
		}
		else if (arr.size() == 2)
		{
			wchr = (arr.at(0) & 0xff) << 8;
			wchr |= (arr.at(1) & 0xff);
		}

		char c = 0;
		int n = wchr;

		if (n >= 0xB0A1 && n <= 0xB0C4) c = 'A';
		if (n >= 0XB0C5 && n <= 0XB2C0) c = 'B';
		if (n >= 0xB2C1 && n <= 0xB4ED) c = 'C';
		if (n >= 0xB4EE && n <= 0xB6E9) c = 'D';
		if (n >= 0xB6EA && n <= 0xB7A1) c = 'E';
		if (n >= 0xB7A2 && n <= 0xB8c0) c = 'F';
		if (n >= 0xB8C1 && n <= 0xB9FD) c = 'G';
		if (n >= 0xB9FE && n <= 0xBBF6) c = 'H';
		if (n >= 0xBBF7 && n <= 0xBFA5) c = 'J';
		if (n >= 0xBFA6 && n <= 0xC0AB) c = 'K';
		if (n >= 0xC0AC && n <= 0xC2E7) c = 'L';
		if (n >= 0xC2E8 && n <= 0xC4C2) c = 'M';
		if (n >= 0xC4C3 && n <= 0xC5B5) c = 'N';
		if (n >= 0xC5B6 && n <= 0xC5BD) c = 'O';
		if (n >= 0xC5BE && n <= 0xC6D9) c = 'P';
		if (n >= 0xC6DA && n <= 0xC8BA) c = 'Q';
		if (n >= 0xC8BB && n <= 0xC8F5) c = 'R';
		if (n >= 0xC8F6 && n <= 0xCBF0) c = 'S';
		if (n >= 0xCBFA && n <= 0xCDD9) c = 'T';
		if (n >= 0xCDDA && n <= 0xCEF3) c = 'W';
		if (n >= 0xCEF4 && n <= 0xD188) c = 'X';
		if (n >= 0xD1B9 && n <= 0xD4D0) c = 'Y';
		if (n >= 0xD4D1 && n <= 0xD7F9) c = 'Z';

		if (c != 0)
		{
			firstLetters.append(c);
		}
	}
	return firstLetters;
}