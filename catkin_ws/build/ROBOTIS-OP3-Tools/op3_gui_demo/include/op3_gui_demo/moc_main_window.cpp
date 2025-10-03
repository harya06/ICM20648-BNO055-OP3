/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-OP3-Tools/op3_gui_demo/include/op3_gui_demo/main_window.hpp"
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
struct qt_meta_stringdata_robotis_op__MainWindow_t {
    QByteArrayData data[35];
    char stringdata0[760];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_robotis_op__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_robotis_op__MainWindow_t qt_meta_stringdata_robotis_op__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 22), // "robotis_op::MainWindow"
QT_MOC_LITERAL(1, 23, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(2, 48, 0), // ""
QT_MOC_LITERAL(3, 49, 27), // "on_button_clear_log_clicked"
QT_MOC_LITERAL(4, 77, 5), // "check"
QT_MOC_LITERAL(5, 83, 27), // "on_button_init_pose_clicked"
QT_MOC_LITERAL(6, 111, 27), // "on_button_init_gyro_clicked"
QT_MOC_LITERAL(7, 139, 31), // "on_button_walking_start_clicked"
QT_MOC_LITERAL(8, 171, 30), // "on_button_walking_stop_clicked"
QT_MOC_LITERAL(9, 202, 31), // "on_button_param_refresh_clicked"
QT_MOC_LITERAL(10, 234, 29), // "on_button_param_apply_clicked"
QT_MOC_LITERAL(11, 264, 28), // "on_button_param_save_clicked"
QT_MOC_LITERAL(12, 293, 30), // "on_checkBox_balance_on_clicked"
QT_MOC_LITERAL(13, 324, 31), // "on_checkBox_balance_off_clicked"
QT_MOC_LITERAL(14, 356, 29), // "on_head_center_button_clicked"
QT_MOC_LITERAL(15, 386, 28), // "on_button_demo_start_clicked"
QT_MOC_LITERAL(16, 415, 27), // "on_button_demo_stop_clicked"
QT_MOC_LITERAL(17, 443, 24), // "on_button_r_kick_clicked"
QT_MOC_LITERAL(18, 468, 24), // "on_button_l_kick_clicked"
QT_MOC_LITERAL(19, 493, 29), // "on_button_getup_front_clicked"
QT_MOC_LITERAL(20, 523, 28), // "on_button_getup_back_clicked"
QT_MOC_LITERAL(21, 552, 17), // "updateLoggingView"
QT_MOC_LITERAL(22, 570, 7), // "setMode"
QT_MOC_LITERAL(23, 578, 22), // "updateCurrentJointMode"
QT_MOC_LITERAL(24, 601, 16), // "std::vector<int>"
QT_MOC_LITERAL(25, 618, 4), // "mode"
QT_MOC_LITERAL(26, 623, 9), // "mode_name"
QT_MOC_LITERAL(27, 633, 16), // "updateHeadAngles"
QT_MOC_LITERAL(28, 650, 3), // "pan"
QT_MOC_LITERAL(29, 654, 4), // "tilt"
QT_MOC_LITERAL(30, 659, 19), // "updateWalkingParams"
QT_MOC_LITERAL(31, 679, 37), // "op3_walking_module_msgs::Walk..."
QT_MOC_LITERAL(32, 717, 6), // "params"
QT_MOC_LITERAL(33, 724, 22), // "walkingCommandShortcut"
QT_MOC_LITERAL(34, 747, 12) // "setHeadAngle"

    },
    "robotis_op::MainWindow\0on_actionAbout_triggered\0"
    "\0on_button_clear_log_clicked\0check\0"
    "on_button_init_pose_clicked\0"
    "on_button_init_gyro_clicked\0"
    "on_button_walking_start_clicked\0"
    "on_button_walking_stop_clicked\0"
    "on_button_param_refresh_clicked\0"
    "on_button_param_apply_clicked\0"
    "on_button_param_save_clicked\0"
    "on_checkBox_balance_on_clicked\0"
    "on_checkBox_balance_off_clicked\0"
    "on_head_center_button_clicked\0"
    "on_button_demo_start_clicked\0"
    "on_button_demo_stop_clicked\0"
    "on_button_r_kick_clicked\0"
    "on_button_l_kick_clicked\0"
    "on_button_getup_front_clicked\0"
    "on_button_getup_back_clicked\0"
    "updateLoggingView\0setMode\0"
    "updateCurrentJointMode\0std::vector<int>\0"
    "mode\0mode_name\0updateHeadAngles\0pan\0"
    "tilt\0updateWalkingParams\0"
    "op3_walking_module_msgs::WalkingParam\0"
    "params\0walkingCommandShortcut\0"
    "setHeadAngle"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_robotis_op__MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      26,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  144,    2, 0x0a /* Public */,
       3,    1,  145,    2, 0x0a /* Public */,
       5,    1,  148,    2, 0x0a /* Public */,
       6,    1,  151,    2, 0x0a /* Public */,
       7,    1,  154,    2, 0x0a /* Public */,
       8,    1,  157,    2, 0x0a /* Public */,
       9,    1,  160,    2, 0x0a /* Public */,
      10,    1,  163,    2, 0x0a /* Public */,
      11,    1,  166,    2, 0x0a /* Public */,
      12,    1,  169,    2, 0x0a /* Public */,
      13,    1,  172,    2, 0x0a /* Public */,
      14,    1,  175,    2, 0x0a /* Public */,
      15,    1,  178,    2, 0x0a /* Public */,
      16,    1,  181,    2, 0x0a /* Public */,
      17,    1,  184,    2, 0x0a /* Public */,
      18,    1,  187,    2, 0x0a /* Public */,
      19,    1,  190,    2, 0x0a /* Public */,
      20,    1,  193,    2, 0x0a /* Public */,
      21,    0,  196,    2, 0x0a /* Public */,
      22,    1,  197,    2, 0x0a /* Public */,
      23,    1,  200,    2, 0x0a /* Public */,
      22,    1,  203,    2, 0x0a /* Public */,
      27,    2,  206,    2, 0x0a /* Public */,
      30,    1,  211,    2, 0x0a /* Public */,
      33,    0,  214,    2, 0x0a /* Public */,
      34,    0,  215,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void, QMetaType::QString,   26,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   28,   29,
    QMetaType::Void, 0x80000000 | 31,   32,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void robotis_op::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_clear_log_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_button_init_pose_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_button_init_gyro_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_button_walking_start_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_button_walking_stop_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_button_param_refresh_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_button_param_apply_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_button_param_save_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_checkBox_balance_on_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_checkBox_balance_off_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->on_head_center_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->on_button_demo_start_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 13: _t->on_button_demo_stop_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->on_button_r_kick_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->on_button_l_kick_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_button_getup_front_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->on_button_getup_back_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->updateLoggingView(); break;
        case 19: _t->setMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 20: _t->updateCurrentJointMode((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 21: _t->setMode((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 22: _t->updateHeadAngles((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 23: _t->updateWalkingParams((*reinterpret_cast< op3_walking_module_msgs::WalkingParam(*)>(_a[1]))); break;
        case 24: _t->walkingCommandShortcut(); break;
        case 25: _t->setHeadAngle(); break;
        default: ;
        }
    }
}

const QMetaObject robotis_op::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_robotis_op__MainWindow.data,
      qt_meta_data_robotis_op__MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *robotis_op::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *robotis_op::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_robotis_op__MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int robotis_op::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 26)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 26;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 26)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 26;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
