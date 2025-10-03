/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-OP3-Tools/op3_gui_demo/include/op3_gui_demo/qnode.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_robotis_op__QNodeOP3_t {
    QByteArrayData data[22];
    char stringdata0[323];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_robotis_op__QNodeOP3_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_robotis_op__QNodeOP3_t qt_meta_stringdata_robotis_op__QNodeOP3 = {
    {
QT_MOC_LITERAL(0, 0, 20), // "robotis_op::QNodeOP3"
QT_MOC_LITERAL(1, 21, 14), // "loggingUpdated"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 11), // "rosShutdown"
QT_MOC_LITERAL(4, 49, 29), // "updateCurrentJointControlMode"
QT_MOC_LITERAL(5, 79, 16), // "std::vector<int>"
QT_MOC_LITERAL(6, 96, 4), // "mode"
QT_MOC_LITERAL(7, 101, 16), // "updateHeadAngles"
QT_MOC_LITERAL(8, 118, 3), // "pan"
QT_MOC_LITERAL(9, 122, 4), // "tilt"
QT_MOC_LITERAL(10, 127, 23), // "updateWalkingParameters"
QT_MOC_LITERAL(11, 151, 37), // "op3_walking_module_msgs::Walk..."
QT_MOC_LITERAL(12, 189, 6), // "params"
QT_MOC_LITERAL(13, 196, 15), // "updateDemoPoint"
QT_MOC_LITERAL(14, 212, 20), // "geometry_msgs::Point"
QT_MOC_LITERAL(15, 233, 5), // "point"
QT_MOC_LITERAL(16, 239, 14), // "updateDemoPose"
QT_MOC_LITERAL(17, 254, 19), // "geometry_msgs::Pose"
QT_MOC_LITERAL(18, 274, 4), // "pose"
QT_MOC_LITERAL(19, 279, 19), // "getJointControlMode"
QT_MOC_LITERAL(20, 299, 10), // "playMotion"
QT_MOC_LITERAL(21, 310, 12) // "motion_index"

    },
    "robotis_op::QNodeOP3\0loggingUpdated\0"
    "\0rosShutdown\0updateCurrentJointControlMode\0"
    "std::vector<int>\0mode\0updateHeadAngles\0"
    "pan\0tilt\0updateWalkingParameters\0"
    "op3_walking_module_msgs::WalkingParam\0"
    "params\0updateDemoPoint\0geometry_msgs::Point\0"
    "point\0updateDemoPose\0geometry_msgs::Pose\0"
    "pose\0getJointControlMode\0playMotion\0"
    "motion_index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_robotis_op__QNodeOP3[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x06 /* Public */,
       3,    0,   60,    2, 0x06 /* Public */,
       4,    1,   61,    2, 0x06 /* Public */,
       7,    2,   64,    2, 0x06 /* Public */,
      10,    1,   69,    2, 0x06 /* Public */,
      13,    1,   72,    2, 0x06 /* Public */,
      16,    1,   75,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      19,    0,   78,    2, 0x0a /* Public */,
      20,    1,   79,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    8,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void, 0x80000000 | 17,   18,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   21,

       0        // eod
};

void robotis_op::QNodeOP3::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QNodeOP3 *_t = static_cast<QNodeOP3 *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->rosShutdown(); break;
        case 2: _t->updateCurrentJointControlMode((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 3: _t->updateHeadAngles((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: _t->updateWalkingParameters((*reinterpret_cast< op3_walking_module_msgs::WalkingParam(*)>(_a[1]))); break;
        case 5: _t->updateDemoPoint((*reinterpret_cast< const geometry_msgs::Point(*)>(_a[1]))); break;
        case 6: _t->updateDemoPose((*reinterpret_cast< const geometry_msgs::Pose(*)>(_a[1]))); break;
        case 7: _t->getJointControlMode(); break;
        case 8: _t->playMotion((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (QNodeOP3::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::loggingUpdated)) {
                *result = 0;
            }
        }
        {
            typedef void (QNodeOP3::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::rosShutdown)) {
                *result = 1;
            }
        }
        {
            typedef void (QNodeOP3::*_t)(std::vector<int> );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::updateCurrentJointControlMode)) {
                *result = 2;
            }
        }
        {
            typedef void (QNodeOP3::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::updateHeadAngles)) {
                *result = 3;
            }
        }
        {
            typedef void (QNodeOP3::*_t)(op3_walking_module_msgs::WalkingParam );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::updateWalkingParameters)) {
                *result = 4;
            }
        }
        {
            typedef void (QNodeOP3::*_t)(const geometry_msgs::Point );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::updateDemoPoint)) {
                *result = 5;
            }
        }
        {
            typedef void (QNodeOP3::*_t)(const geometry_msgs::Pose );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QNodeOP3::updateDemoPose)) {
                *result = 6;
            }
        }
    }
}

const QMetaObject robotis_op::QNodeOP3::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_robotis_op__QNodeOP3.data,
      qt_meta_data_robotis_op__QNodeOP3,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *robotis_op::QNodeOP3::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *robotis_op::QNodeOP3::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_robotis_op__QNodeOP3.stringdata0))
        return static_cast<void*>(const_cast< QNodeOP3*>(this));
    return QThread::qt_metacast(_clname);
}

int robotis_op::QNodeOP3::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void robotis_op::QNodeOP3::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void robotis_op::QNodeOP3::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void robotis_op::QNodeOP3::updateCurrentJointControlMode(std::vector<int> _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void robotis_op::QNodeOP3::updateHeadAngles(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void robotis_op::QNodeOP3::updateWalkingParameters(op3_walking_module_msgs::WalkingParam _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void robotis_op::QNodeOP3::updateDemoPoint(const geometry_msgs::Point _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void robotis_op::QNodeOP3::updateDemoPose(const geometry_msgs::Pose _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_END_MOC_NAMESPACE
