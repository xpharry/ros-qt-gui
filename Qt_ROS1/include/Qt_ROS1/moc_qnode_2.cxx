/****************************************************************************
** Meta object code from reading C++ file 'qnode_2.hpp'
**
** Created: Thu Jul 17 13:32:37 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qnode_2.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode_2.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Qt_ROS1__QNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,
      39,   33,   15,   15, 0x05,
      60,   15,   15,   15, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Qt_ROS1__QNode[] = {
    "Qt_ROS1::QNode\0\0loggingUpdated()\0image\0"
    "updateImage(QPixmap)\0rosShutdown()\0"
};

void Qt_ROS1::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->updateImage((*reinterpret_cast< QPixmap(*)>(_a[1]))); break;
        case 2: _t->rosShutdown(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Qt_ROS1::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Qt_ROS1::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Qt_ROS1__QNode,
      qt_meta_data_Qt_ROS1__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Qt_ROS1::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Qt_ROS1::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Qt_ROS1::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Qt_ROS1__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int Qt_ROS1::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void Qt_ROS1::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Qt_ROS1::QNode::updateImage(QPixmap _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Qt_ROS1::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
