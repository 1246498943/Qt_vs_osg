/****************************************************************************
** Meta object code from reading C++ file 'osgRender.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../osgRender.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'osgRender.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_osgRender_t {
    QByteArrayData data[7];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_osgRender_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_osgRender_t qt_meta_stringdata_osgRender = {
    {
QT_MOC_LITERAL(0, 0, 9), // "osgRender"
QT_MOC_LITERAL(1, 10, 19), // "onAddAscFileClicked"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 9), // "isChecked"
QT_MOC_LITERAL(4, 41, 20), // "onAddOstrFileClicked"
QT_MOC_LITERAL(5, 62, 17), // "onAddModelClicked"
QT_MOC_LITERAL(6, 80, 13) // "onAddDrawAble"

    },
    "osgRender\0onAddAscFileClicked\0\0isChecked\0"
    "onAddOstrFileClicked\0onAddModelClicked\0"
    "onAddDrawAble"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_osgRender[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x08 /* Private */,
       4,    1,   37,    2, 0x08 /* Private */,
       5,    1,   40,    2, 0x08 /* Private */,
       6,    1,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void osgRender::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        osgRender *_t = static_cast<osgRender *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onAddAscFileClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->onAddOstrFileClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->onAddModelClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->onAddDrawAble((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject osgRender::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_osgRender.data,
      qt_meta_data_osgRender,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *osgRender::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *osgRender::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_osgRender.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int osgRender::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
