/****************************************************************************
** Meta object code from reading C++ file 'OSGScanModel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../OSGScanModel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'OSGScanModel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OSGScanModel_t {
    QByteArrayData data[7];
    char stringdata0[69];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OSGScanModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OSGScanModel_t qt_meta_stringdata_OSGScanModel = {
    {
QT_MOC_LITERAL(0, 0, 12), // "OSGScanModel"
QT_MOC_LITERAL(1, 13, 8), // "homeView"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 21), // "updateCameraDirection"
QT_MOC_LITERAL(4, 45, 9), // "osg::Vec3"
QT_MOC_LITERAL(5, 55, 6), // "center"
QT_MOC_LITERAL(6, 62, 6) // "normal"

    },
    "OSGScanModel\0homeView\0\0updateCameraDirection\0"
    "osg::Vec3\0center\0normal"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OSGScanModel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x06 /* Public */,
       3,    2,   25,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4, 0x80000000 | 4,    5,    6,

       0        // eod
};

void OSGScanModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OSGScanModel *_t = static_cast<OSGScanModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->homeView(); break;
        case 1: _t->updateCameraDirection((*reinterpret_cast< osg::Vec3(*)>(_a[1])),(*reinterpret_cast< osg::Vec3(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (OSGScanModel::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OSGScanModel::homeView)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (OSGScanModel::*_t)(osg::Vec3 , osg::Vec3 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OSGScanModel::updateCameraDirection)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject OSGScanModel::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_OSGScanModel.data,
      qt_meta_data_OSGScanModel,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *OSGScanModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OSGScanModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OSGScanModel.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "osg::MatrixTransform"))
        return static_cast< osg::MatrixTransform*>(this);
    return QObject::qt_metacast(_clname);
}

int OSGScanModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void OSGScanModel::homeView()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void OSGScanModel::updateCameraDirection(osg::Vec3 _t1, osg::Vec3 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE