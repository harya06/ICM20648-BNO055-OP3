/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-OP3-Tools/op3_offset_tuner_client/include/op3_offset_tuner_client/main_window.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_op3_offset_tuner_client__MainWindow_t {
    QByteArrayData data[20];
    char stringdata0[379];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_op3_offset_tuner_client__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_op3_offset_tuner_client__MainWindow_t qt_meta_stringdata_op3_offset_tuner_client__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 35), // "op3_offset_tuner_client::Main..."
QT_MOC_LITERAL(1, 36, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(2, 61, 0), // ""
QT_MOC_LITERAL(3, 62, 22), // "on_save_button_clicked"
QT_MOC_LITERAL(4, 85, 5), // "check"
QT_MOC_LITERAL(5, 91, 25), // "on_refresh_button_clicked"
QT_MOC_LITERAL(6, 117, 25), // "on_inipose_button_clicked"
QT_MOC_LITERAL(7, 143, 6), // "checck"
QT_MOC_LITERAL(8, 150, 17), // "updateLoggingView"
QT_MOC_LITERAL(9, 168, 24), // "updateJointOffsetSpinbox"
QT_MOC_LITERAL(10, 193, 46), // "op3_offset_tuner_msgs::JointO..."
QT_MOC_LITERAL(11, 240, 3), // "msg"
QT_MOC_LITERAL(12, 244, 19), // "changedSpinBoxValue"
QT_MOC_LITERAL(13, 264, 12), // "q_joint_name"
QT_MOC_LITERAL(14, 277, 21), // "clickedTorqueCheckbox"
QT_MOC_LITERAL(15, 299, 8), // "QWidget*"
QT_MOC_LITERAL(16, 308, 6), // "widget"
QT_MOC_LITERAL(17, 315, 24), // "clickedAllTorqueOnButton"
QT_MOC_LITERAL(18, 340, 12), // "button_group"
QT_MOC_LITERAL(19, 353, 25) // "clickedAllTorqueOffButton"

    },
    "op3_offset_tuner_client::MainWindow\0"
    "on_actionAbout_triggered\0\0"
    "on_save_button_clicked\0check\0"
    "on_refresh_button_clicked\0"
    "on_inipose_button_clicked\0checck\0"
    "updateLoggingView\0updateJointOffsetSpinbox\0"
    "op3_offset_tuner_msgs::JointOffsetPositionData\0"
    "msg\0changedSpinBoxValue\0q_joint_name\0"
    "clickedTorqueCheckbox\0QWidget*\0widget\0"
    "clickedAllTorqueOnButton\0button_group\0"
    "clickedAllTorqueOffButton"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_op3_offset_tuner_client__MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x0a /* Public */,
       3,    1,   65,    2, 0x0a /* Public */,
       5,    1,   68,    2, 0x0a /* Public */,
       6,    1,   71,    2, 0x0a /* Public */,
       8,    0,   74,    2, 0x0a /* Public */,
       9,    1,   75,    2, 0x0a /* Public */,
      12,    1,   78,    2, 0x0a /* Public */,
      14,    1,   81,    2, 0x0a /* Public */,
      17,    1,   84,    2, 0x0a /* Public */,
      19,    1,   87,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, QMetaType::QObjectStar,   18,
    QMetaType::Void, QMetaType::QObjectStar,   18,

       0        // eod
};

void op3_offset_tuner_client::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_save_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_refresh_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_inipose_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->updateLoggingView(); break;
        case 5: _t->updateJointOffsetSpinbox((*reinterpret_cast< op3_offset_tuner_msgs::JointOffsetPositionData(*)>(_a[1]))); break;
        case 6: _t->changedSpinBoxValue((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->clickedTorqueCheckbox((*reinterpret_cast< QWidget*(*)>(_a[1]))); break;
        case 8: _t->clickedAllTorqueOnButton((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        case 9: _t->clickedAllTorqueOffButton((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject op3_offset_tuner_client::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_op3_offset_tuner_client__MainWindow.data,
      qt_meta_data_op3_offset_tuner_client__MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *op3_offset_tuner_client::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *op3_offset_tuner_client::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_op3_offset_tuner_client__MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int op3_offset_tuner_client::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
