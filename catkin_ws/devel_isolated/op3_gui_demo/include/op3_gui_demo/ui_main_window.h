/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "preview_walking_form.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_log;
    QGridLayout *gridLayout_3;
    QPushButton *button_clear_log;
    QListView *view_logging;
    QWidget *widget_control;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox_lidar;
    QGridLayout *gridLayout_5;
    QWidget *widget_ft_base_module;
    QGridLayout *gridLayout_7;
    QPushButton *button_init_pose;
    QFrame *line;
    QTabWidget *tabWidget_control;
    QWidget *tab_mode;
    QVBoxLayout *verticalLayout;
    QWidget *widget_mode_preset;
    QWidget *widget_mode;
    QSpacerItem *verticalSpacer_2;
    QWidget *tab_walking_module;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox_init_pose;
    QHBoxLayout *horizontalLayout_3;
    QFrame *frame_2;
    QGridLayout *gridLayout_4;
    QDoubleSpinBox *dSpinBox_init_offset_z;
    QLabel *label_13;
    QDoubleSpinBox *dSpinBox_init_offset_x;
    QDoubleSpinBox *dSpinBox_init_offset_y;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_34;
    QFrame *frame;
    QGridLayout *gridLayout_8;
    QDoubleSpinBox *dSpinBox_init_offset_roll;
    QDoubleSpinBox *dSpinBox_init_offset_pitch;
    QLabel *label_10;
    QLabel *label_15;
    QLabel *label_14;
    QDoubleSpinBox *dSpinBox_init_offset_yaw;
    QLabel *label_33;
    QDoubleSpinBox *dSpinBox_hip_pitch_offset;
    QGroupBox *groupBox_walking_param;
    QHBoxLayout *horizontalLayout_4;
    QFrame *frame_3;
    QGridLayout *gridLayout_9;
    QLabel *label_16;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *label_23;
    QCheckBox *checkBox_move_aim_on;
    QCheckBox *checkBox_move_aim_off;
    QDoubleSpinBox *dSpinBox_step_fb_ratio;
    QDoubleSpinBox *dSpinBox_dsp_ratio;
    QDoubleSpinBox *dSpinBox_period_time;
    QFrame *frame_4;
    QGridLayout *gridLayout_10;
    QLabel *label_17;
    QLabel *label_21;
    QLabel *label_20;
    QDoubleSpinBox *dSpinBox_x_move_amplitude;
    QLabel *label_22;
    QDoubleSpinBox *dSpinBox_y_move_amplitude;
    QDoubleSpinBox *dSpinBox_z_move_amplitude;
    QDoubleSpinBox *dSpinBox_a_move_amplitude;
    QGroupBox *groupBox_balance_control;
    QGridLayout *gridLayout_12;
    QFrame *frame_5;
    QGridLayout *gridLayout_11;
    QLabel *label_24;
    QCheckBox *checkBox_balance_on;
    QLabel *label_26;
    QCheckBox *checkBox_balance_off;
    QLabel *label_25;
    QLabel *label_27;
    QLabel *label_28;
    QDoubleSpinBox *dSpinBox_hip_roll_gain;
    QDoubleSpinBox *dSpinBox_knee_gain;
    QDoubleSpinBox *dSpinBox_ankle_roll_gain;
    QDoubleSpinBox *dSpinBox_ankle_pitch_gain;
    QFrame *frame_6;
    QGridLayout *gridLayout_13;
    QLabel *label_31;
    QLabel *label_32;
    QLabel *label_29;
    QDoubleSpinBox *dSpinBox_z_swap_amplitude;
    QDoubleSpinBox *dSpinBox_arm_swing_gain;
    QLabel *label_30;
    QDoubleSpinBox *dSpinBox_y_swap_amplitude;
    QDoubleSpinBox *dSpinBox_pelvis_offset;
    QPushButton *button_init_gyro;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *button_param_refresh;
    QPushButton *button_param_save;
    QPushButton *button_param_apply;
    QFrame *line_5;
    QGroupBox *groupBox_walking_command;
    QHBoxLayout *horizontalLayout;
    QPushButton *button_walking_start;
    QPushButton *button_walking_stop;
    QWidget *tab_head_control_module;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_6;
    QSlider *head_tilt_slider;
    QSpinBox *head_pan_spinbox;
    QLabel *label_5;
    QPushButton *head_center_button;
    QSpinBox *head_tilt_spinbox;
    QLabel *label_6;
    QLabel *label_3;
    QSlider *head_pan_slider;
    QLabel *label_4;
    QSpacerItem *verticalSpacer_4;
    QFrame *line_4;
    QLabel *label_7;
    QLabel *label_8;
    QWidget *tab_action_module;
    QVBoxLayout *verticalLayout_8;
    QScrollArea *scrollArea;
    QWidget *scroll_widget_motion;
    QWidget *tab_demo;
    QGridLayout *gridLayout_14;
    QPushButton *button_l_kick;
    QPushButton *button_demo_start;
    QPushButton *button_demo_stop;
    QPushButton *button_getup_back;
    QSpacerItem *verticalSpacer_3;
    QPushButton *button_r_kick;
    QPushButton *button_getup_front;
    QFrame *line_6;
    QWidget *tab_online_walking_module;
    PreviewWalkingForm *widget_preview_walking;
    QSpacerItem *verticalSpacer;
    QSpacerItem *horizontalSpacer;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;
    QButtonGroup *buttonGroup_move_aim_on;
    QButtonGroup *buttonGroup_balance;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QStringLiteral("MainWindowDesign"));
        MainWindowDesign->resize(1477, 976);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QStringLiteral("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QStringLiteral("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QStringLiteral("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QStringLiteral("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_log = new QGroupBox(tab_status);
        groupBox_log->setObjectName(QStringLiteral("groupBox_log"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_log->sizePolicy().hasHeightForWidth());
        groupBox_log->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_log);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        button_clear_log = new QPushButton(groupBox_log);
        button_clear_log->setObjectName(QStringLiteral("button_clear_log"));
        button_clear_log->setMinimumSize(QSize(300, 0));

        gridLayout_3->addWidget(button_clear_log, 1, 0, 1, 1);

        view_logging = new QListView(groupBox_log);
        view_logging->setObjectName(QStringLiteral("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_log);

        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        widget_control = new QWidget(centralwidget);
        widget_control->setObjectName(QStringLiteral("widget_control"));
        verticalLayout_3 = new QVBoxLayout(widget_control);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox_lidar = new QGroupBox(widget_control);
        groupBox_lidar->setObjectName(QStringLiteral("groupBox_lidar"));
        gridLayout_5 = new QGridLayout(groupBox_lidar);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        widget_ft_base_module = new QWidget(groupBox_lidar);
        widget_ft_base_module->setObjectName(QStringLiteral("widget_ft_base_module"));
        widget_ft_base_module->setEnabled(false);
        gridLayout_7 = new QGridLayout(widget_ft_base_module);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        gridLayout_7->setContentsMargins(0, 0, 0, 0);

        gridLayout_5->addWidget(widget_ft_base_module, 2, 1, 1, 3);

        button_init_pose = new QPushButton(groupBox_lidar);
        button_init_pose->setObjectName(QStringLiteral("button_init_pose"));

        gridLayout_5->addWidget(button_init_pose, 1, 1, 1, 3);


        verticalLayout_3->addWidget(groupBox_lidar);

        line = new QFrame(widget_control);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line);

        tabWidget_control = new QTabWidget(widget_control);
        tabWidget_control->setObjectName(QStringLiteral("tabWidget_control"));
        tabWidget_control->setEnabled(true);
        tab_mode = new QWidget();
        tab_mode->setObjectName(QStringLiteral("tab_mode"));
        verticalLayout = new QVBoxLayout(tab_mode);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        widget_mode_preset = new QWidget(tab_mode);
        widget_mode_preset->setObjectName(QStringLiteral("widget_mode_preset"));

        verticalLayout->addWidget(widget_mode_preset);

        widget_mode = new QWidget(tab_mode);
        widget_mode->setObjectName(QStringLiteral("widget_mode"));

        verticalLayout->addWidget(widget_mode);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        tabWidget_control->addTab(tab_mode, QString());
        tab_walking_module = new QWidget();
        tab_walking_module->setObjectName(QStringLiteral("tab_walking_module"));
        tab_walking_module->setEnabled(true);
        verticalLayout_5 = new QVBoxLayout(tab_walking_module);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        groupBox_init_pose = new QGroupBox(tab_walking_module);
        groupBox_init_pose->setObjectName(QStringLiteral("groupBox_init_pose"));
        horizontalLayout_3 = new QHBoxLayout(groupBox_init_pose);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        frame_2 = new QFrame(groupBox_init_pose);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        gridLayout_4 = new QGridLayout(frame_2);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        dSpinBox_init_offset_z = new QDoubleSpinBox(frame_2);
        dSpinBox_init_offset_z->setObjectName(QStringLiteral("dSpinBox_init_offset_z"));
        dSpinBox_init_offset_z->setDecimals(3);
        dSpinBox_init_offset_z->setMinimum(-1);
        dSpinBox_init_offset_z->setMaximum(1);
        dSpinBox_init_offset_z->setSingleStep(0.001);
        dSpinBox_init_offset_z->setValue(0.02);

        gridLayout_4->addWidget(dSpinBox_init_offset_z, 2, 1, 1, 2);

        label_13 = new QLabel(frame_2);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_4->addWidget(label_13, 2, 0, 1, 1);

        dSpinBox_init_offset_x = new QDoubleSpinBox(frame_2);
        dSpinBox_init_offset_x->setObjectName(QStringLiteral("dSpinBox_init_offset_x"));
        dSpinBox_init_offset_x->setDecimals(3);
        dSpinBox_init_offset_x->setMinimum(-1);
        dSpinBox_init_offset_x->setMaximum(1);
        dSpinBox_init_offset_x->setSingleStep(0.001);
        dSpinBox_init_offset_x->setValue(-0.01);

        gridLayout_4->addWidget(dSpinBox_init_offset_x, 0, 1, 1, 2);

        dSpinBox_init_offset_y = new QDoubleSpinBox(frame_2);
        dSpinBox_init_offset_y->setObjectName(QStringLiteral("dSpinBox_init_offset_y"));
        dSpinBox_init_offset_y->setDecimals(3);
        dSpinBox_init_offset_y->setMinimum(-1);
        dSpinBox_init_offset_y->setMaximum(1);
        dSpinBox_init_offset_y->setSingleStep(0.001);
        dSpinBox_init_offset_y->setValue(0.005);

        gridLayout_4->addWidget(dSpinBox_init_offset_y, 1, 1, 1, 2);

        label_11 = new QLabel(frame_2);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_4->addWidget(label_11, 0, 0, 1, 1);

        label_12 = new QLabel(frame_2);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_4->addWidget(label_12, 1, 0, 1, 1);

        label_34 = new QLabel(frame_2);
        label_34->setObjectName(QStringLiteral("label_34"));

        gridLayout_4->addWidget(label_34, 3, 0, 1, 1);


        horizontalLayout_3->addWidget(frame_2);

        frame = new QFrame(groupBox_init_pose);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_8 = new QGridLayout(frame);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        dSpinBox_init_offset_roll = new QDoubleSpinBox(frame);
        dSpinBox_init_offset_roll->setObjectName(QStringLiteral("dSpinBox_init_offset_roll"));
        dSpinBox_init_offset_roll->setDecimals(1);
        dSpinBox_init_offset_roll->setMinimum(-180);
        dSpinBox_init_offset_roll->setMaximum(180);

        gridLayout_8->addWidget(dSpinBox_init_offset_roll, 0, 1, 1, 2);

        dSpinBox_init_offset_pitch = new QDoubleSpinBox(frame);
        dSpinBox_init_offset_pitch->setObjectName(QStringLiteral("dSpinBox_init_offset_pitch"));
        dSpinBox_init_offset_pitch->setDecimals(1);
        dSpinBox_init_offset_pitch->setMinimum(-180);
        dSpinBox_init_offset_pitch->setMaximum(180);

        gridLayout_8->addWidget(dSpinBox_init_offset_pitch, 1, 1, 1, 2);

        label_10 = new QLabel(frame);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout_8->addWidget(label_10, 0, 0, 1, 1);

        label_15 = new QLabel(frame);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_8->addWidget(label_15, 2, 0, 1, 1);

        label_14 = new QLabel(frame);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_8->addWidget(label_14, 1, 0, 1, 1);

        dSpinBox_init_offset_yaw = new QDoubleSpinBox(frame);
        dSpinBox_init_offset_yaw->setObjectName(QStringLiteral("dSpinBox_init_offset_yaw"));
        dSpinBox_init_offset_yaw->setDecimals(1);
        dSpinBox_init_offset_yaw->setMinimum(-180);
        dSpinBox_init_offset_yaw->setMaximum(180);

        gridLayout_8->addWidget(dSpinBox_init_offset_yaw, 2, 1, 1, 2);

        label_33 = new QLabel(frame);
        label_33->setObjectName(QStringLiteral("label_33"));

        gridLayout_8->addWidget(label_33, 3, 0, 1, 1);

        dSpinBox_hip_pitch_offset = new QDoubleSpinBox(frame);
        dSpinBox_hip_pitch_offset->setObjectName(QStringLiteral("dSpinBox_hip_pitch_offset"));
        dSpinBox_hip_pitch_offset->setDecimals(1);
        dSpinBox_hip_pitch_offset->setMinimum(-180);
        dSpinBox_hip_pitch_offset->setMaximum(180);
        dSpinBox_hip_pitch_offset->setValue(13);

        gridLayout_8->addWidget(dSpinBox_hip_pitch_offset, 3, 1, 1, 2);


        horizontalLayout_3->addWidget(frame);


        verticalLayout_5->addWidget(groupBox_init_pose);

        groupBox_walking_param = new QGroupBox(tab_walking_module);
        groupBox_walking_param->setObjectName(QStringLiteral("groupBox_walking_param"));
        horizontalLayout_4 = new QHBoxLayout(groupBox_walking_param);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        frame_3 = new QFrame(groupBox_walking_param);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        gridLayout_9 = new QGridLayout(frame_3);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        label_16 = new QLabel(frame_3);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_9->addWidget(label_16, 0, 0, 1, 1);

        label_18 = new QLabel(frame_3);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_9->addWidget(label_18, 1, 0, 1, 1);

        label_19 = new QLabel(frame_3);
        label_19->setObjectName(QStringLiteral("label_19"));

        gridLayout_9->addWidget(label_19, 2, 0, 1, 1);

        label_23 = new QLabel(frame_3);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_9->addWidget(label_23, 3, 0, 1, 1);

        checkBox_move_aim_on = new QCheckBox(frame_3);
        buttonGroup_move_aim_on = new QButtonGroup(MainWindowDesign);
        buttonGroup_move_aim_on->setObjectName(QStringLiteral("buttonGroup_move_aim_on"));
        buttonGroup_move_aim_on->addButton(checkBox_move_aim_on);
        checkBox_move_aim_on->setObjectName(QStringLiteral("checkBox_move_aim_on"));

        gridLayout_9->addWidget(checkBox_move_aim_on, 3, 1, 1, 1);

        checkBox_move_aim_off = new QCheckBox(frame_3);
        buttonGroup_move_aim_on->addButton(checkBox_move_aim_off);
        checkBox_move_aim_off->setObjectName(QStringLiteral("checkBox_move_aim_off"));
        checkBox_move_aim_off->setChecked(true);

        gridLayout_9->addWidget(checkBox_move_aim_off, 3, 2, 1, 1);

        dSpinBox_step_fb_ratio = new QDoubleSpinBox(frame_3);
        dSpinBox_step_fb_ratio->setObjectName(QStringLiteral("dSpinBox_step_fb_ratio"));
        dSpinBox_step_fb_ratio->setMaximum(1);
        dSpinBox_step_fb_ratio->setSingleStep(0.1);
        dSpinBox_step_fb_ratio->setValue(0.28);

        gridLayout_9->addWidget(dSpinBox_step_fb_ratio, 2, 1, 1, 2);

        dSpinBox_dsp_ratio = new QDoubleSpinBox(frame_3);
        dSpinBox_dsp_ratio->setObjectName(QStringLiteral("dSpinBox_dsp_ratio"));
        dSpinBox_dsp_ratio->setMaximum(1);
        dSpinBox_dsp_ratio->setSingleStep(0.1);
        dSpinBox_dsp_ratio->setValue(0.1);

        gridLayout_9->addWidget(dSpinBox_dsp_ratio, 1, 1, 1, 2);

        dSpinBox_period_time = new QDoubleSpinBox(frame_3);
        dSpinBox_period_time->setObjectName(QStringLiteral("dSpinBox_period_time"));
        dSpinBox_period_time->setDecimals(0);
        dSpinBox_period_time->setMaximum(5000);
        dSpinBox_period_time->setSingleStep(10);
        dSpinBox_period_time->setValue(600);

        gridLayout_9->addWidget(dSpinBox_period_time, 0, 1, 1, 2);


        horizontalLayout_4->addWidget(frame_3);

        frame_4 = new QFrame(groupBox_walking_param);
        frame_4->setObjectName(QStringLiteral("frame_4"));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);
        gridLayout_10 = new QGridLayout(frame_4);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        label_17 = new QLabel(frame_4);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_10->addWidget(label_17, 0, 0, 1, 1);

        label_21 = new QLabel(frame_4);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_10->addWidget(label_21, 2, 0, 1, 1);

        label_20 = new QLabel(frame_4);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_10->addWidget(label_20, 1, 0, 1, 1);

        dSpinBox_x_move_amplitude = new QDoubleSpinBox(frame_4);
        dSpinBox_x_move_amplitude->setObjectName(QStringLiteral("dSpinBox_x_move_amplitude"));
        dSpinBox_x_move_amplitude->setDecimals(3);
        dSpinBox_x_move_amplitude->setMinimum(-1);
        dSpinBox_x_move_amplitude->setMaximum(1);
        dSpinBox_x_move_amplitude->setSingleStep(0.001);

        gridLayout_10->addWidget(dSpinBox_x_move_amplitude, 0, 1, 1, 1);

        label_22 = new QLabel(frame_4);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_10->addWidget(label_22, 3, 0, 1, 1);

        dSpinBox_y_move_amplitude = new QDoubleSpinBox(frame_4);
        dSpinBox_y_move_amplitude->setObjectName(QStringLiteral("dSpinBox_y_move_amplitude"));
        dSpinBox_y_move_amplitude->setDecimals(3);
        dSpinBox_y_move_amplitude->setMinimum(-1);
        dSpinBox_y_move_amplitude->setMaximum(1);
        dSpinBox_y_move_amplitude->setSingleStep(0.001);

        gridLayout_10->addWidget(dSpinBox_y_move_amplitude, 1, 1, 1, 1);

        dSpinBox_z_move_amplitude = new QDoubleSpinBox(frame_4);
        dSpinBox_z_move_amplitude->setObjectName(QStringLiteral("dSpinBox_z_move_amplitude"));
        dSpinBox_z_move_amplitude->setDecimals(3);
        dSpinBox_z_move_amplitude->setMinimum(-1);
        dSpinBox_z_move_amplitude->setMaximum(1);
        dSpinBox_z_move_amplitude->setSingleStep(0.001);
        dSpinBox_z_move_amplitude->setValue(0.04);

        gridLayout_10->addWidget(dSpinBox_z_move_amplitude, 2, 1, 1, 1);

        dSpinBox_a_move_amplitude = new QDoubleSpinBox(frame_4);
        dSpinBox_a_move_amplitude->setObjectName(QStringLiteral("dSpinBox_a_move_amplitude"));
        dSpinBox_a_move_amplitude->setDecimals(1);
        dSpinBox_a_move_amplitude->setMinimum(-180);
        dSpinBox_a_move_amplitude->setMaximum(180);

        gridLayout_10->addWidget(dSpinBox_a_move_amplitude, 3, 1, 1, 1);


        horizontalLayout_4->addWidget(frame_4);


        verticalLayout_5->addWidget(groupBox_walking_param);

        groupBox_balance_control = new QGroupBox(tab_walking_module);
        groupBox_balance_control->setObjectName(QStringLiteral("groupBox_balance_control"));
        gridLayout_12 = new QGridLayout(groupBox_balance_control);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        frame_5 = new QFrame(groupBox_balance_control);
        frame_5->setObjectName(QStringLiteral("frame_5"));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);
        gridLayout_11 = new QGridLayout(frame_5);
        gridLayout_11->setObjectName(QStringLiteral("gridLayout_11"));
        label_24 = new QLabel(frame_5);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_11->addWidget(label_24, 1, 0, 1, 1);

        checkBox_balance_on = new QCheckBox(frame_5);
        buttonGroup_balance = new QButtonGroup(MainWindowDesign);
        buttonGroup_balance->setObjectName(QStringLiteral("buttonGroup_balance"));
        buttonGroup_balance->addButton(checkBox_balance_on);
        checkBox_balance_on->setObjectName(QStringLiteral("checkBox_balance_on"));

        gridLayout_11->addWidget(checkBox_balance_on, 0, 1, 1, 1);

        label_26 = new QLabel(frame_5);
        label_26->setObjectName(QStringLiteral("label_26"));

        gridLayout_11->addWidget(label_26, 0, 0, 1, 1);

        checkBox_balance_off = new QCheckBox(frame_5);
        buttonGroup_balance->addButton(checkBox_balance_off);
        checkBox_balance_off->setObjectName(QStringLiteral("checkBox_balance_off"));
        checkBox_balance_off->setChecked(true);

        gridLayout_11->addWidget(checkBox_balance_off, 0, 2, 1, 1);

        label_25 = new QLabel(frame_5);
        label_25->setObjectName(QStringLiteral("label_25"));

        gridLayout_11->addWidget(label_25, 2, 0, 1, 1);

        label_27 = new QLabel(frame_5);
        label_27->setObjectName(QStringLiteral("label_27"));

        gridLayout_11->addWidget(label_27, 3, 0, 1, 1);

        label_28 = new QLabel(frame_5);
        label_28->setObjectName(QStringLiteral("label_28"));

        gridLayout_11->addWidget(label_28, 4, 0, 1, 1);

        dSpinBox_hip_roll_gain = new QDoubleSpinBox(frame_5);
        dSpinBox_hip_roll_gain->setObjectName(QStringLiteral("dSpinBox_hip_roll_gain"));
        dSpinBox_hip_roll_gain->setMaximum(1);
        dSpinBox_hip_roll_gain->setSingleStep(0.1);
        dSpinBox_hip_roll_gain->setValue(0.5);

        gridLayout_11->addWidget(dSpinBox_hip_roll_gain, 1, 1, 1, 2);

        dSpinBox_knee_gain = new QDoubleSpinBox(frame_5);
        dSpinBox_knee_gain->setObjectName(QStringLiteral("dSpinBox_knee_gain"));
        dSpinBox_knee_gain->setMaximum(1);
        dSpinBox_knee_gain->setSingleStep(0.1);
        dSpinBox_knee_gain->setValue(0.3);

        gridLayout_11->addWidget(dSpinBox_knee_gain, 2, 1, 1, 2);

        dSpinBox_ankle_roll_gain = new QDoubleSpinBox(frame_5);
        dSpinBox_ankle_roll_gain->setObjectName(QStringLiteral("dSpinBox_ankle_roll_gain"));
        dSpinBox_ankle_roll_gain->setMaximum(1);
        dSpinBox_ankle_roll_gain->setSingleStep(0.1);
        dSpinBox_ankle_roll_gain->setValue(1);

        gridLayout_11->addWidget(dSpinBox_ankle_roll_gain, 3, 1, 1, 2);

        dSpinBox_ankle_pitch_gain = new QDoubleSpinBox(frame_5);
        dSpinBox_ankle_pitch_gain->setObjectName(QStringLiteral("dSpinBox_ankle_pitch_gain"));
        dSpinBox_ankle_pitch_gain->setMaximum(1);
        dSpinBox_ankle_pitch_gain->setSingleStep(0.1);
        dSpinBox_ankle_pitch_gain->setValue(0.9);

        gridLayout_11->addWidget(dSpinBox_ankle_pitch_gain, 4, 1, 1, 2);


        gridLayout_12->addWidget(frame_5, 0, 0, 1, 1);

        frame_6 = new QFrame(groupBox_balance_control);
        frame_6->setObjectName(QStringLiteral("frame_6"));
        frame_6->setFrameShape(QFrame::StyledPanel);
        frame_6->setFrameShadow(QFrame::Raised);
        gridLayout_13 = new QGridLayout(frame_6);
        gridLayout_13->setObjectName(QStringLiteral("gridLayout_13"));
        label_31 = new QLabel(frame_6);
        label_31->setObjectName(QStringLiteral("label_31"));

        gridLayout_13->addWidget(label_31, 4, 0, 1, 1);

        label_32 = new QLabel(frame_6);
        label_32->setObjectName(QStringLiteral("label_32"));

        gridLayout_13->addWidget(label_32, 5, 0, 1, 1);

        label_29 = new QLabel(frame_6);
        label_29->setObjectName(QStringLiteral("label_29"));

        gridLayout_13->addWidget(label_29, 2, 0, 1, 1);

        dSpinBox_z_swap_amplitude = new QDoubleSpinBox(frame_6);
        dSpinBox_z_swap_amplitude->setObjectName(QStringLiteral("dSpinBox_z_swap_amplitude"));
        dSpinBox_z_swap_amplitude->setDecimals(3);
        dSpinBox_z_swap_amplitude->setMinimum(-1);
        dSpinBox_z_swap_amplitude->setMaximum(1);
        dSpinBox_z_swap_amplitude->setSingleStep(0.001);
        dSpinBox_z_swap_amplitude->setValue(0.005);

        gridLayout_13->addWidget(dSpinBox_z_swap_amplitude, 3, 1, 1, 1);

        dSpinBox_arm_swing_gain = new QDoubleSpinBox(frame_6);
        dSpinBox_arm_swing_gain->setObjectName(QStringLiteral("dSpinBox_arm_swing_gain"));
        dSpinBox_arm_swing_gain->setMaximum(2);
        dSpinBox_arm_swing_gain->setSingleStep(0.1);
        dSpinBox_arm_swing_gain->setValue(1.5);

        gridLayout_13->addWidget(dSpinBox_arm_swing_gain, 4, 1, 1, 1);

        label_30 = new QLabel(frame_6);
        label_30->setObjectName(QStringLiteral("label_30"));

        gridLayout_13->addWidget(label_30, 3, 0, 1, 1);

        dSpinBox_y_swap_amplitude = new QDoubleSpinBox(frame_6);
        dSpinBox_y_swap_amplitude->setObjectName(QStringLiteral("dSpinBox_y_swap_amplitude"));
        dSpinBox_y_swap_amplitude->setDecimals(3);
        dSpinBox_y_swap_amplitude->setMinimum(-1);
        dSpinBox_y_swap_amplitude->setMaximum(1);
        dSpinBox_y_swap_amplitude->setSingleStep(0.001);
        dSpinBox_y_swap_amplitude->setValue(0.02);

        gridLayout_13->addWidget(dSpinBox_y_swap_amplitude, 2, 1, 1, 1);

        dSpinBox_pelvis_offset = new QDoubleSpinBox(frame_6);
        dSpinBox_pelvis_offset->setObjectName(QStringLiteral("dSpinBox_pelvis_offset"));
        dSpinBox_pelvis_offset->setDecimals(1);
        dSpinBox_pelvis_offset->setMinimum(-180);
        dSpinBox_pelvis_offset->setMaximum(180);
        dSpinBox_pelvis_offset->setValue(3);

        gridLayout_13->addWidget(dSpinBox_pelvis_offset, 5, 1, 1, 1);

        button_init_gyro = new QPushButton(frame_6);
        button_init_gyro->setObjectName(QStringLiteral("button_init_gyro"));

        gridLayout_13->addWidget(button_init_gyro, 0, 0, 1, 1);


        gridLayout_12->addWidget(frame_6, 0, 1, 1, 1);


        verticalLayout_5->addWidget(groupBox_balance_control);

        widget = new QWidget(tab_walking_module);
        widget->setObjectName(QStringLiteral("widget"));
        horizontalLayout_7 = new QHBoxLayout(widget);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        button_param_refresh = new QPushButton(widget);
        button_param_refresh->setObjectName(QStringLiteral("button_param_refresh"));

        horizontalLayout_7->addWidget(button_param_refresh);

        button_param_save = new QPushButton(widget);
        button_param_save->setObjectName(QStringLiteral("button_param_save"));

        horizontalLayout_7->addWidget(button_param_save);

        button_param_apply = new QPushButton(widget);
        button_param_apply->setObjectName(QStringLiteral("button_param_apply"));

        horizontalLayout_7->addWidget(button_param_apply);


        verticalLayout_5->addWidget(widget);

        line_5 = new QFrame(tab_walking_module);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);

        verticalLayout_5->addWidget(line_5);

        groupBox_walking_command = new QGroupBox(tab_walking_module);
        groupBox_walking_command->setObjectName(QStringLiteral("groupBox_walking_command"));
        horizontalLayout = new QHBoxLayout(groupBox_walking_command);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        button_walking_start = new QPushButton(groupBox_walking_command);
        button_walking_start->setObjectName(QStringLiteral("button_walking_start"));

        horizontalLayout->addWidget(button_walking_start);

        button_walking_stop = new QPushButton(groupBox_walking_command);
        button_walking_stop->setObjectName(QStringLiteral("button_walking_stop"));

        horizontalLayout->addWidget(button_walking_stop);


        verticalLayout_5->addWidget(groupBox_walking_command);

        tabWidget_control->addTab(tab_walking_module, QString());
        tab_head_control_module = new QWidget();
        tab_head_control_module->setObjectName(QStringLiteral("tab_head_control_module"));
        tab_head_control_module->setEnabled(true);
        verticalLayout_7 = new QVBoxLayout(tab_head_control_module);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        groupBox_2 = new QGroupBox(tab_head_control_module);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        gridLayout_6 = new QGridLayout(groupBox_2);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        head_tilt_slider = new QSlider(groupBox_2);
        head_tilt_slider->setObjectName(QStringLiteral("head_tilt_slider"));
        head_tilt_slider->setMaximumSize(QSize(400, 16777215));
        head_tilt_slider->setMinimum(-70);
        head_tilt_slider->setMaximum(30);
        head_tilt_slider->setSingleStep(3);
        head_tilt_slider->setOrientation(Qt::Horizontal);

        gridLayout_6->addWidget(head_tilt_slider, 1, 3, 1, 1);

        head_pan_spinbox = new QSpinBox(groupBox_2);
        head_pan_spinbox->setObjectName(QStringLiteral("head_pan_spinbox"));
        head_pan_spinbox->setMinimum(-90);
        head_pan_spinbox->setMaximum(90);
        head_pan_spinbox->setSingleStep(3);

        gridLayout_6->addWidget(head_pan_spinbox, 0, 5, 1, 2);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setMaximumSize(QSize(20, 16777215));

        gridLayout_6->addWidget(label_5, 0, 2, 1, 1);

        head_center_button = new QPushButton(groupBox_2);
        head_center_button->setObjectName(QStringLiteral("head_center_button"));

        gridLayout_6->addWidget(head_center_button, 2, 5, 1, 2);

        head_tilt_spinbox = new QSpinBox(groupBox_2);
        head_tilt_spinbox->setObjectName(QStringLiteral("head_tilt_spinbox"));
        head_tilt_spinbox->setMinimum(-90);
        head_tilt_spinbox->setMaximum(90);
        head_tilt_spinbox->setSingleStep(3);

        gridLayout_6->addWidget(head_tilt_spinbox, 1, 5, 1, 2);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setMaximumSize(QSize(20, 16777215));

        gridLayout_6->addWidget(label_6, 0, 4, 1, 1);

        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_6->addWidget(label_3, 0, 0, 1, 1);

        head_pan_slider = new QSlider(groupBox_2);
        head_pan_slider->setObjectName(QStringLiteral("head_pan_slider"));
        head_pan_slider->setMaximumSize(QSize(400, 16777215));
        head_pan_slider->setMinimum(-90);
        head_pan_slider->setMaximum(90);
        head_pan_slider->setSingleStep(3);
        head_pan_slider->setOrientation(Qt::Horizontal);

        gridLayout_6->addWidget(head_pan_slider, 0, 3, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_6->addWidget(label_4, 1, 0, 1, 1);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_6->addItem(verticalSpacer_4, 3, 0, 1, 1);

        line_4 = new QFrame(groupBox_2);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::VLine);
        line_4->setFrameShadow(QFrame::Sunken);

        gridLayout_6->addWidget(line_4, 0, 1, 2, 1);

        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_6->addWidget(label_7, 1, 2, 1, 1);

        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_6->addWidget(label_8, 1, 4, 1, 1);


        verticalLayout_7->addWidget(groupBox_2);

        tabWidget_control->addTab(tab_head_control_module, QString());
        tab_action_module = new QWidget();
        tab_action_module->setObjectName(QStringLiteral("tab_action_module"));
        verticalLayout_8 = new QVBoxLayout(tab_action_module);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        scrollArea = new QScrollArea(tab_action_module);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scroll_widget_motion = new QWidget();
        scroll_widget_motion->setObjectName(QStringLiteral("scroll_widget_motion"));
        scroll_widget_motion->setGeometry(QRect(0, 0, 684, 677));
        scrollArea->setWidget(scroll_widget_motion);

        verticalLayout_8->addWidget(scrollArea);

        tabWidget_control->addTab(tab_action_module, QString());
        tab_demo = new QWidget();
        tab_demo->setObjectName(QStringLiteral("tab_demo"));
        gridLayout_14 = new QGridLayout(tab_demo);
        gridLayout_14->setObjectName(QStringLiteral("gridLayout_14"));
        button_l_kick = new QPushButton(tab_demo);
        button_l_kick->setObjectName(QStringLiteral("button_l_kick"));

        gridLayout_14->addWidget(button_l_kick, 2, 1, 1, 1);

        button_demo_start = new QPushButton(tab_demo);
        button_demo_start->setObjectName(QStringLiteral("button_demo_start"));

        gridLayout_14->addWidget(button_demo_start, 0, 0, 1, 1);

        button_demo_stop = new QPushButton(tab_demo);
        button_demo_stop->setObjectName(QStringLiteral("button_demo_stop"));

        gridLayout_14->addWidget(button_demo_stop, 0, 1, 1, 1);

        button_getup_back = new QPushButton(tab_demo);
        button_getup_back->setObjectName(QStringLiteral("button_getup_back"));

        gridLayout_14->addWidget(button_getup_back, 3, 1, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_14->addItem(verticalSpacer_3, 4, 0, 1, 1);

        button_r_kick = new QPushButton(tab_demo);
        button_r_kick->setObjectName(QStringLiteral("button_r_kick"));

        gridLayout_14->addWidget(button_r_kick, 2, 0, 1, 1);

        button_getup_front = new QPushButton(tab_demo);
        button_getup_front->setObjectName(QStringLiteral("button_getup_front"));

        gridLayout_14->addWidget(button_getup_front, 3, 0, 1, 1);

        line_6 = new QFrame(tab_demo);
        line_6->setObjectName(QStringLiteral("line_6"));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);

        gridLayout_14->addWidget(line_6, 1, 0, 1, 2);

        tabWidget_control->addTab(tab_demo, QString());
        tab_online_walking_module = new QWidget();
        tab_online_walking_module->setObjectName(QStringLiteral("tab_online_walking_module"));
        widget_preview_walking = new PreviewWalkingForm(tab_online_walking_module);
        widget_preview_walking->setObjectName(QStringLiteral("widget_preview_walking"));
        widget_preview_walking->setGeometry(QRect(0, 0, 691, 650));
        widget_preview_walking->setMinimumSize(QSize(0, 650));
        tabWidget_control->addTab(tab_online_walking_module, QString());

        verticalLayout_3->addWidget(tabWidget_control);

        verticalSpacer = new QSpacerItem(20, 446, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        horizontalSpacer = new QSpacerItem(697, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_3->addItem(horizontalSpacer);


        hboxLayout->addWidget(widget_control);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1477, 19));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        QWidget::setTabOrder(tab_manager, view_logging);
        QWidget::setTabOrder(view_logging, button_clear_log);
        QWidget::setTabOrder(button_clear_log, button_init_pose);
        QWidget::setTabOrder(button_init_pose, tabWidget_control);
        QWidget::setTabOrder(tabWidget_control, button_walking_start);
        QWidget::setTabOrder(button_walking_start, button_walking_stop);
        QWidget::setTabOrder(button_walking_stop, dSpinBox_init_offset_x);
        QWidget::setTabOrder(dSpinBox_init_offset_x, dSpinBox_init_offset_y);
        QWidget::setTabOrder(dSpinBox_init_offset_y, dSpinBox_init_offset_z);
        QWidget::setTabOrder(dSpinBox_init_offset_z, dSpinBox_init_offset_roll);
        QWidget::setTabOrder(dSpinBox_init_offset_roll, dSpinBox_init_offset_pitch);
        QWidget::setTabOrder(dSpinBox_init_offset_pitch, dSpinBox_init_offset_yaw);
        QWidget::setTabOrder(dSpinBox_init_offset_yaw, dSpinBox_hip_pitch_offset);
        QWidget::setTabOrder(dSpinBox_hip_pitch_offset, dSpinBox_period_time);
        QWidget::setTabOrder(dSpinBox_period_time, dSpinBox_dsp_ratio);
        QWidget::setTabOrder(dSpinBox_dsp_ratio, dSpinBox_step_fb_ratio);
        QWidget::setTabOrder(dSpinBox_step_fb_ratio, checkBox_move_aim_on);
        QWidget::setTabOrder(checkBox_move_aim_on, checkBox_move_aim_off);
        QWidget::setTabOrder(checkBox_move_aim_off, dSpinBox_x_move_amplitude);
        QWidget::setTabOrder(dSpinBox_x_move_amplitude, dSpinBox_y_move_amplitude);
        QWidget::setTabOrder(dSpinBox_y_move_amplitude, dSpinBox_z_move_amplitude);
        QWidget::setTabOrder(dSpinBox_z_move_amplitude, dSpinBox_a_move_amplitude);
        QWidget::setTabOrder(dSpinBox_a_move_amplitude, checkBox_balance_on);
        QWidget::setTabOrder(checkBox_balance_on, checkBox_balance_off);
        QWidget::setTabOrder(checkBox_balance_off, dSpinBox_hip_roll_gain);
        QWidget::setTabOrder(dSpinBox_hip_roll_gain, dSpinBox_knee_gain);
        QWidget::setTabOrder(dSpinBox_knee_gain, dSpinBox_ankle_roll_gain);
        QWidget::setTabOrder(dSpinBox_ankle_roll_gain, dSpinBox_ankle_pitch_gain);
        QWidget::setTabOrder(dSpinBox_ankle_pitch_gain, dSpinBox_y_swap_amplitude);
        QWidget::setTabOrder(dSpinBox_y_swap_amplitude, dSpinBox_z_swap_amplitude);
        QWidget::setTabOrder(dSpinBox_z_swap_amplitude, dSpinBox_arm_swing_gain);
        QWidget::setTabOrder(dSpinBox_arm_swing_gain, dSpinBox_pelvis_offset);
        QWidget::setTabOrder(dSpinBox_pelvis_offset, button_param_refresh);
        QWidget::setTabOrder(button_param_refresh, button_param_save);
        QWidget::setTabOrder(button_param_save, button_param_apply);
        QWidget::setTabOrder(button_param_apply, head_pan_slider);
        QWidget::setTabOrder(head_pan_slider, head_pan_spinbox);
        QWidget::setTabOrder(head_pan_spinbox, head_tilt_slider);
        QWidget::setTabOrder(head_tilt_slider, head_tilt_spinbox);
        QWidget::setTabOrder(head_tilt_spinbox, head_center_button);
        QWidget::setTabOrder(head_center_button, scrollArea);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(head_pan_slider, SIGNAL(valueChanged(int)), head_pan_spinbox, SLOT(setValue(int)));
        QObject::connect(head_pan_spinbox, SIGNAL(valueChanged(int)), head_pan_slider, SLOT(setValue(int)));
        QObject::connect(head_tilt_slider, SIGNAL(valueChanged(int)), head_tilt_spinbox, SLOT(setValue(int)));
        QObject::connect(head_tilt_spinbox, SIGNAL(valueChanged(int)), head_tilt_slider, SLOT(setValue(int)));

        tab_manager->setCurrentIndex(0);
        tabWidget_control->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "OP3 Demo", 0));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0));
        groupBox_log->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0));
        button_clear_log->setText(QApplication::translate("MainWindowDesign", "Clear", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0));
        groupBox_lidar->setTitle(QApplication::translate("MainWindowDesign", "Basic Control", 0));
        button_init_pose->setText(QApplication::translate("MainWindowDesign", "Robot Init Pose", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_mode), QApplication::translate("MainWindowDesign", "Mode Control", 0));
        groupBox_init_pose->setTitle(QApplication::translate("MainWindowDesign", "Initial Pose", 0));
        dSpinBox_init_offset_z->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        label_13->setText(QApplication::translate("MainWindowDesign", "Z", 0));
        dSpinBox_init_offset_x->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        dSpinBox_init_offset_y->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        label_11->setText(QApplication::translate("MainWindowDesign", "X", 0));
        label_12->setText(QApplication::translate("MainWindowDesign", "Y", 0));
        label_34->setText(QString());
        dSpinBox_init_offset_roll->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        dSpinBox_init_offset_pitch->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        label_10->setText(QApplication::translate("MainWindowDesign", "Roll", 0));
        label_15->setText(QApplication::translate("MainWindowDesign", "Yaw", 0));
        label_14->setText(QApplication::translate("MainWindowDesign", "Pitch", 0));
        dSpinBox_init_offset_yaw->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        label_33->setText(QApplication::translate("MainWindowDesign", "Hip Pitch Offset", 0));
        dSpinBox_hip_pitch_offset->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        groupBox_walking_param->setTitle(QApplication::translate("MainWindowDesign", "Walking Parameter", 0));
        label_16->setText(QApplication::translate("MainWindowDesign", "Period Time", 0));
        label_18->setText(QApplication::translate("MainWindowDesign", "DSP Ratio", 0));
        label_19->setText(QApplication::translate("MainWindowDesign", "Step FB Ratio", 0));
        label_23->setText(QApplication::translate("MainWindowDesign", "Move Aim", 0));
        checkBox_move_aim_on->setText(QApplication::translate("MainWindowDesign", "On", 0));
        checkBox_move_aim_off->setText(QApplication::translate("MainWindowDesign", "Off", 0));
        dSpinBox_period_time->setSuffix(QApplication::translate("MainWindowDesign", " \343\216\263", 0));
        label_17->setText(QApplication::translate("MainWindowDesign", "X Move Amplitude", 0));
        label_21->setText(QApplication::translate("MainWindowDesign", "Z Move Amplitude", 0));
        label_20->setText(QApplication::translate("MainWindowDesign", "Y Move Amplitude", 0));
        dSpinBox_x_move_amplitude->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        label_22->setText(QApplication::translate("MainWindowDesign", "Yaw Move Amplitude", 0));
        dSpinBox_y_move_amplitude->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        dSpinBox_z_move_amplitude->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        dSpinBox_a_move_amplitude->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        groupBox_balance_control->setTitle(QApplication::translate("MainWindowDesign", "Balance Control", 0));
        label_24->setText(QApplication::translate("MainWindowDesign", "Hip Roll Gain", 0));
        checkBox_balance_on->setText(QApplication::translate("MainWindowDesign", "On", 0));
        label_26->setText(QApplication::translate("MainWindowDesign", "Balance", 0));
        checkBox_balance_off->setText(QApplication::translate("MainWindowDesign", "Off", 0));
        label_25->setText(QApplication::translate("MainWindowDesign", "Knee Gain", 0));
        label_27->setText(QApplication::translate("MainWindowDesign", "Ankle Roll Gain", 0));
        label_28->setText(QApplication::translate("MainWindowDesign", "Ankle Pitch Gain", 0));
        label_31->setText(QApplication::translate("MainWindowDesign", "Arm Swing Gain", 0));
        label_32->setText(QApplication::translate("MainWindowDesign", "Pelvis Offset", 0));
        label_29->setText(QApplication::translate("MainWindowDesign", "Y Swap Amplitude", 0));
        dSpinBox_z_swap_amplitude->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        label_30->setText(QApplication::translate("MainWindowDesign", "Z Swap Amplitude", 0));
        dSpinBox_y_swap_amplitude->setSuffix(QApplication::translate("MainWindowDesign", " \357\275\215", 0));
        dSpinBox_pelvis_offset->setSuffix(QApplication::translate("MainWindowDesign", "  \357\276\237", 0));
        button_init_gyro->setText(QApplication::translate("MainWindowDesign", "Init Gyro", 0));
        button_param_refresh->setText(QApplication::translate("MainWindowDesign", "Refresh", 0));
        button_param_save->setText(QApplication::translate("MainWindowDesign", "Save", 0));
        button_param_apply->setText(QApplication::translate("MainWindowDesign", "Apply", 0));
        button_param_apply->setShortcut(QApplication::translate("MainWindowDesign", "\\", 0));
        groupBox_walking_command->setTitle(QApplication::translate("MainWindowDesign", "Walking Command", 0));
        button_walking_start->setText(QApplication::translate("MainWindowDesign", "Start", 0));
        button_walking_start->setShortcut(QApplication::translate("MainWindowDesign", "F12", 0));
        button_walking_stop->setText(QApplication::translate("MainWindowDesign", "Stop", 0));
        button_walking_stop->setShortcut(QApplication::translate("MainWindowDesign", "Esc", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_walking_module), QApplication::translate("MainWindowDesign", "Walking", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindowDesign", "Head Joint", 0));
        head_pan_spinbox->setSuffix(QApplication::translate("MainWindowDesign", " \313\232", 0));
        label_5->setText(QApplication::translate("MainWindowDesign", "\342\227\200", 0));
        head_center_button->setText(QApplication::translate("MainWindowDesign", "Center", 0));
        head_tilt_spinbox->setSuffix(QApplication::translate("MainWindowDesign", " \313\232", 0));
        label_6->setText(QApplication::translate("MainWindowDesign", " \342\226\266 ", 0));
        label_3->setText(QApplication::translate("MainWindowDesign", "Head Pan", 0));
        label_4->setText(QApplication::translate("MainWindowDesign", "Head Tilt", 0));
        label_7->setText(QApplication::translate("MainWindowDesign", "\342\226\274", 0));
        label_8->setText(QApplication::translate("MainWindowDesign", "\342\226\262", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_head_control_module), QApplication::translate("MainWindowDesign", "Head Control", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_action_module), QApplication::translate("MainWindowDesign", "Motion", 0));
        button_l_kick->setText(QApplication::translate("MainWindowDesign", "Left Kick", 0));
        button_demo_start->setText(QApplication::translate("MainWindowDesign", "Start Tracking", 0));
        button_demo_stop->setText(QApplication::translate("MainWindowDesign", "Stop Tracking", 0));
        button_getup_back->setText(QApplication::translate("MainWindowDesign", "Get up(Back)", 0));
        button_r_kick->setText(QApplication::translate("MainWindowDesign", "Right Kick", 0));
        button_getup_front->setText(QApplication::translate("MainWindowDesign", "Get up(Front)", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_demo), QApplication::translate("MainWindowDesign", "Demo", 0));
        tabWidget_control->setTabText(tabWidget_control->indexOf(tab_online_walking_module), QApplication::translate("MainWindowDesign", "Online Walking", 0));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
