/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.hpp'
**
** Created: Thu Jul 17 13:26:12 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Qt_ROS1__Main_Window[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      28,   22,   21,   21, 0x0a,
      66,   60,   21,   21, 0x0a,
     114,   22,   21,   21, 0x0a,
     137,   22,   21,   21, 0x0a,
     162,   22,   21,   21, 0x0a,
     188,   22,   21,   21, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Qt_ROS1__Main_Window[] = {
    "Qt_ROS1::Main_Window\0\0check\0"
    "on_button_connect_clicked(bool)\0state\0"
    "on_checkbox_use_environment_2_stateChanged(int)\0"
    "on_Pb_Up_clicked(bool)\0on_Pb_Down_clicked(bool)\0"
    "on_Pb_Right_clicked(bool)\0"
    "on_Pb_Left_clicked(bool)\0"
};

void Qt_ROS1::Main_Window::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Main_Window *_t = static_cast<Main_Window *>(_o);
        switch (_id) {
        case 0: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_checkbox_use_environment_2_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_Pb_Up_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_Pb_Down_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_Pb_Right_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_Pb_Left_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Qt_ROS1::Main_Window::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Qt_ROS1::Main_Window::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Qt_ROS1__Main_Window,
      qt_meta_data_Qt_ROS1__Main_Window, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Qt_ROS1::Main_Window::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Qt_ROS1::Main_Window::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Qt_ROS1::Main_Window::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Qt_ROS1__Main_Window))
        return static_cast<void*>(const_cast< Main_Window*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int Qt_ROS1::Main_Window::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
