/********************************************************************************
** Form generated from reading UI file 'preview_walking_form.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PREVIEW_WALKING_FORM_H
#define UI_PREVIEW_WALKING_FORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PreviewWalkingForm
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_10;
    QDoubleSpinBox *dSpinBox_marker_pos_x;
    QLabel *label_16;
    QLabel *label_14;
    QDoubleSpinBox *dSpinBox_marker_ori_p;
    QPushButton *button_marker_clear;
    QLabel *label_19;
    QFrame *line_6;
    QDoubleSpinBox *dSpinBox_marker_ori_y;
    QDoubleSpinBox *dSpinBox_marker_pos_y;
    QFrame *line_7;
    QLabel *label_17;
    QDoubleSpinBox *dSpinBox_marker_pos_z;
    QLabel *label_18;
    QLabel *label_15;
    QPushButton *button_marker_set;
    QDoubleSpinBox *dSpinBox_marker_ori_r;
    QGroupBox *walkin_param;
    QGridLayout *gridLayout_20;
    QLabel *label_46;
    QLabel *label_43;
    QDoubleSpinBox *dSpinBox_body_offset_z;
    QDoubleSpinBox *dSpinBox_foot_distance;
    QDoubleSpinBox *dSpinBox_dsp_ratio;
    QPushButton *button_set_walking_param;
    QPushButton *button_send_body_offset;
    QLabel *label_44;
    QLabel *label_47;
    QLabel *label_45;
    QDoubleSpinBox *dSpinBox_zmp_offset_y;
    QDoubleSpinBox *dSpinBox_body_offset_y;
    QLabel *label_64;
    QLabel *label_66;
    QDoubleSpinBox *dSpinBox_body_offset_x;
    QLabel *label_63;
    QDoubleSpinBox *dSpinBox_zmp_offset_x;
    QDoubleSpinBox *dSpinBox_foot_height_max;
    QLabel *label_65;
    QSpacerItem *verticalSpacer_3;
    QDoubleSpinBox *dSpinBox_lipm_height;
    QPushButton *button_send_foot_distance;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox_clear_path;
    QCheckBox *checkBox_clear_marker;
    QPushButton *button_footstep_plan;
    QPushButton *button_footstep_clear;
    QPushButton *button_footstep_go;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_19;
    QLabel *label_41;
    QPushButton *button_p_walking_right;
    QPushButton *button_p_walking_forward;
    QLabel *label_38;
    QPushButton *button_p_walking_left;
    QLabel *label_37;
    QPushButton *button_p_walking_backward;
    QDoubleSpinBox *dSpinBox_p_walking_step_angle;
    QPushButton *button_p_walking_stop;
    QLabel *label_39;
    QLabel *label_40;
    QDoubleSpinBox *dSpinBox_p_walking_step_length;
    QDoubleSpinBox *dSpinBox_p_walking_step_num;
    QPushButton *button_p_walking_turn_l;
    QDoubleSpinBox *dSpinBox_p_walking_side_length;
    QDoubleSpinBox *dSpinBox_p_walking_step_time;
    QPushButton *button_p_walking_turn_r;
    QLabel *label_42;
    QComboBox *comboBox_p_walking_start_leg;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *button_p_walking_init_pose;
    QPushButton *button_p_walking_balance_on;
    QPushButton *button_p_walking_balance_off;

    void setupUi(QWidget *PreviewWalkingForm)
    {
        if (PreviewWalkingForm->objectName().isEmpty())
            PreviewWalkingForm->setObjectName(QStringLiteral("PreviewWalkingForm"));
        PreviewWalkingForm->resize(861, 653);
        gridLayout = new QGridLayout(PreviewWalkingForm);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        groupBox_6 = new QGroupBox(PreviewWalkingForm);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        gridLayout_10 = new QGridLayout(groupBox_6);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        dSpinBox_marker_pos_x = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_pos_x->setObjectName(QStringLiteral("dSpinBox_marker_pos_x"));
        dSpinBox_marker_pos_x->setDecimals(3);
        dSpinBox_marker_pos_x->setMinimum(-10);
        dSpinBox_marker_pos_x->setMaximum(10);
        dSpinBox_marker_pos_x->setSingleStep(0.005);

        gridLayout_10->addWidget(dSpinBox_marker_pos_x, 1, 0, 1, 1);

        label_16 = new QLabel(groupBox_6);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_16, 0, 2, 1, 1);

        label_14 = new QLabel(groupBox_6);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_14, 0, 0, 1, 1);

        dSpinBox_marker_ori_p = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_ori_p->setObjectName(QStringLiteral("dSpinBox_marker_ori_p"));
        dSpinBox_marker_ori_p->setDecimals(1);
        dSpinBox_marker_ori_p->setMinimum(-180);
        dSpinBox_marker_ori_p->setMaximum(180);
        dSpinBox_marker_ori_p->setSingleStep(5);

        gridLayout_10->addWidget(dSpinBox_marker_ori_p, 1, 5, 1, 1);

        button_marker_clear = new QPushButton(groupBox_6);
        button_marker_clear->setObjectName(QStringLiteral("button_marker_clear"));

        gridLayout_10->addWidget(button_marker_clear, 1, 8, 1, 1);

        label_19 = new QLabel(groupBox_6);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_19, 0, 6, 1, 1);

        line_6 = new QFrame(groupBox_6);
        line_6->setObjectName(QStringLiteral("line_6"));
        line_6->setFrameShape(QFrame::VLine);
        line_6->setFrameShadow(QFrame::Sunken);

        gridLayout_10->addWidget(line_6, 0, 3, 2, 1);

        dSpinBox_marker_ori_y = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_ori_y->setObjectName(QStringLiteral("dSpinBox_marker_ori_y"));
        dSpinBox_marker_ori_y->setDecimals(1);
        dSpinBox_marker_ori_y->setMinimum(-180);
        dSpinBox_marker_ori_y->setMaximum(180);
        dSpinBox_marker_ori_y->setSingleStep(5);

        gridLayout_10->addWidget(dSpinBox_marker_ori_y, 1, 6, 1, 1);

        dSpinBox_marker_pos_y = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_pos_y->setObjectName(QStringLiteral("dSpinBox_marker_pos_y"));
        dSpinBox_marker_pos_y->setDecimals(3);
        dSpinBox_marker_pos_y->setMinimum(-10);
        dSpinBox_marker_pos_y->setMaximum(10);
        dSpinBox_marker_pos_y->setSingleStep(0.005);

        gridLayout_10->addWidget(dSpinBox_marker_pos_y, 1, 1, 1, 1);

        line_7 = new QFrame(groupBox_6);
        line_7->setObjectName(QStringLiteral("line_7"));
        line_7->setFrameShape(QFrame::VLine);
        line_7->setFrameShadow(QFrame::Sunken);

        gridLayout_10->addWidget(line_7, 0, 7, 2, 1);

        label_17 = new QLabel(groupBox_6);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_17, 0, 4, 1, 1);

        dSpinBox_marker_pos_z = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_pos_z->setObjectName(QStringLiteral("dSpinBox_marker_pos_z"));
        dSpinBox_marker_pos_z->setDecimals(3);
        dSpinBox_marker_pos_z->setMinimum(-10);
        dSpinBox_marker_pos_z->setMaximum(10);
        dSpinBox_marker_pos_z->setSingleStep(0.005);

        gridLayout_10->addWidget(dSpinBox_marker_pos_z, 1, 2, 1, 1);

        label_18 = new QLabel(groupBox_6);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_18, 0, 5, 1, 1);

        label_15 = new QLabel(groupBox_6);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_15, 0, 1, 1, 1);

        button_marker_set = new QPushButton(groupBox_6);
        button_marker_set->setObjectName(QStringLiteral("button_marker_set"));

        gridLayout_10->addWidget(button_marker_set, 0, 8, 1, 1);

        dSpinBox_marker_ori_r = new QDoubleSpinBox(groupBox_6);
        dSpinBox_marker_ori_r->setObjectName(QStringLiteral("dSpinBox_marker_ori_r"));
        dSpinBox_marker_ori_r->setDecimals(1);
        dSpinBox_marker_ori_r->setMinimum(-180);
        dSpinBox_marker_ori_r->setMaximum(180);
        dSpinBox_marker_ori_r->setSingleStep(5);

        gridLayout_10->addWidget(dSpinBox_marker_ori_r, 1, 4, 1, 1);


        gridLayout->addWidget(groupBox_6, 2, 0, 1, 4);

        walkin_param = new QGroupBox(PreviewWalkingForm);
        walkin_param->setObjectName(QStringLiteral("walkin_param"));
        walkin_param->setMinimumSize(QSize(0, 420));
        gridLayout_20 = new QGridLayout(walkin_param);
        gridLayout_20->setObjectName(QStringLiteral("gridLayout_20"));
        label_46 = new QLabel(walkin_param);
        label_46->setObjectName(QStringLiteral("label_46"));

        gridLayout_20->addWidget(label_46, 3, 0, 1, 1);

        label_43 = new QLabel(walkin_param);
        label_43->setObjectName(QStringLiteral("label_43"));

        gridLayout_20->addWidget(label_43, 0, 0, 1, 1);

        dSpinBox_body_offset_z = new QDoubleSpinBox(walkin_param);
        dSpinBox_body_offset_z->setObjectName(QStringLiteral("dSpinBox_body_offset_z"));
        dSpinBox_body_offset_z->setDecimals(3);
        dSpinBox_body_offset_z->setMinimum(-99);
        dSpinBox_body_offset_z->setSingleStep(0.005);

        gridLayout_20->addWidget(dSpinBox_body_offset_z, 8, 1, 1, 1);

        dSpinBox_foot_distance = new QDoubleSpinBox(walkin_param);
        dSpinBox_foot_distance->setObjectName(QStringLiteral("dSpinBox_foot_distance"));
        dSpinBox_foot_distance->setDecimals(3);
        dSpinBox_foot_distance->setMinimum(-99);
        dSpinBox_foot_distance->setSingleStep(0.005);
        dSpinBox_foot_distance->setValue(0.07);

        gridLayout_20->addWidget(dSpinBox_foot_distance, 10, 1, 1, 1);

        dSpinBox_dsp_ratio = new QDoubleSpinBox(walkin_param);
        dSpinBox_dsp_ratio->setObjectName(QStringLiteral("dSpinBox_dsp_ratio"));
        dSpinBox_dsp_ratio->setMaximum(1);
        dSpinBox_dsp_ratio->setSingleStep(0.1);
        dSpinBox_dsp_ratio->setValue(0.2);

        gridLayout_20->addWidget(dSpinBox_dsp_ratio, 0, 1, 1, 1);

        button_set_walking_param = new QPushButton(walkin_param);
        button_set_walking_param->setObjectName(QStringLiteral("button_set_walking_param"));

        gridLayout_20->addWidget(button_set_walking_param, 5, 1, 1, 1);

        button_send_body_offset = new QPushButton(walkin_param);
        button_send_body_offset->setObjectName(QStringLiteral("button_send_body_offset"));

        gridLayout_20->addWidget(button_send_body_offset, 9, 1, 1, 1);

        label_44 = new QLabel(walkin_param);
        label_44->setObjectName(QStringLiteral("label_44"));

        gridLayout_20->addWidget(label_44, 1, 0, 1, 1);

        label_47 = new QLabel(walkin_param);
        label_47->setObjectName(QStringLiteral("label_47"));

        gridLayout_20->addWidget(label_47, 4, 0, 1, 1);

        label_45 = new QLabel(walkin_param);
        label_45->setObjectName(QStringLiteral("label_45"));

        gridLayout_20->addWidget(label_45, 2, 0, 1, 1);

        dSpinBox_zmp_offset_y = new QDoubleSpinBox(walkin_param);
        dSpinBox_zmp_offset_y->setObjectName(QStringLiteral("dSpinBox_zmp_offset_y"));
        dSpinBox_zmp_offset_y->setDecimals(3);
        dSpinBox_zmp_offset_y->setMinimum(-100);
        dSpinBox_zmp_offset_y->setMaximum(100);
        dSpinBox_zmp_offset_y->setSingleStep(0.005);
        dSpinBox_zmp_offset_y->setValue(0);

        gridLayout_20->addWidget(dSpinBox_zmp_offset_y, 4, 1, 1, 1);

        dSpinBox_body_offset_y = new QDoubleSpinBox(walkin_param);
        dSpinBox_body_offset_y->setObjectName(QStringLiteral("dSpinBox_body_offset_y"));
        dSpinBox_body_offset_y->setDecimals(3);
        dSpinBox_body_offset_y->setMinimum(-99);
        dSpinBox_body_offset_y->setSingleStep(0.005);

        gridLayout_20->addWidget(dSpinBox_body_offset_y, 7, 1, 1, 1);

        label_64 = new QLabel(walkin_param);
        label_64->setObjectName(QStringLiteral("label_64"));

        gridLayout_20->addWidget(label_64, 7, 0, 1, 1);

        label_66 = new QLabel(walkin_param);
        label_66->setObjectName(QStringLiteral("label_66"));

        gridLayout_20->addWidget(label_66, 10, 0, 1, 1);

        dSpinBox_body_offset_x = new QDoubleSpinBox(walkin_param);
        dSpinBox_body_offset_x->setObjectName(QStringLiteral("dSpinBox_body_offset_x"));
        dSpinBox_body_offset_x->setDecimals(3);
        dSpinBox_body_offset_x->setMinimum(-99);
        dSpinBox_body_offset_x->setSingleStep(0.005);

        gridLayout_20->addWidget(dSpinBox_body_offset_x, 6, 1, 1, 1);

        label_63 = new QLabel(walkin_param);
        label_63->setObjectName(QStringLiteral("label_63"));

        gridLayout_20->addWidget(label_63, 6, 0, 1, 1);

        dSpinBox_zmp_offset_x = new QDoubleSpinBox(walkin_param);
        dSpinBox_zmp_offset_x->setObjectName(QStringLiteral("dSpinBox_zmp_offset_x"));
        dSpinBox_zmp_offset_x->setDecimals(3);
        dSpinBox_zmp_offset_x->setMinimum(-100);
        dSpinBox_zmp_offset_x->setMaximum(100);
        dSpinBox_zmp_offset_x->setSingleStep(0.01);
        dSpinBox_zmp_offset_x->setValue(0);

        gridLayout_20->addWidget(dSpinBox_zmp_offset_x, 3, 1, 1, 1);

        dSpinBox_foot_height_max = new QDoubleSpinBox(walkin_param);
        dSpinBox_foot_height_max->setObjectName(QStringLiteral("dSpinBox_foot_height_max"));
        dSpinBox_foot_height_max->setMaximum(0.2);
        dSpinBox_foot_height_max->setSingleStep(0.01);
        dSpinBox_foot_height_max->setValue(0.05);

        gridLayout_20->addWidget(dSpinBox_foot_height_max, 2, 1, 1, 1);

        label_65 = new QLabel(walkin_param);
        label_65->setObjectName(QStringLiteral("label_65"));

        gridLayout_20->addWidget(label_65, 8, 0, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_20->addItem(verticalSpacer_3, 12, 1, 1, 1);

        dSpinBox_lipm_height = new QDoubleSpinBox(walkin_param);
        dSpinBox_lipm_height->setObjectName(QStringLiteral("dSpinBox_lipm_height"));
        dSpinBox_lipm_height->setEnabled(false);
        dSpinBox_lipm_height->setMaximum(100);
        dSpinBox_lipm_height->setSingleStep(0.1);
        dSpinBox_lipm_height->setValue(0.12);

        gridLayout_20->addWidget(dSpinBox_lipm_height, 1, 1, 1, 1);

        button_send_foot_distance = new QPushButton(walkin_param);
        button_send_foot_distance->setObjectName(QStringLiteral("button_send_foot_distance"));

        gridLayout_20->addWidget(button_send_foot_distance, 11, 1, 1, 1);


        gridLayout->addWidget(walkin_param, 1, 2, 1, 2);

        groupBox_2 = new QGroupBox(PreviewWalkingForm);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        horizontalLayout_2 = new QHBoxLayout(groupBox_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        checkBox_clear_path = new QCheckBox(groupBox_2);
        checkBox_clear_path->setObjectName(QStringLiteral("checkBox_clear_path"));
        checkBox_clear_path->setChecked(true);

        horizontalLayout_2->addWidget(checkBox_clear_path);

        checkBox_clear_marker = new QCheckBox(groupBox_2);
        checkBox_clear_marker->setObjectName(QStringLiteral("checkBox_clear_marker"));

        horizontalLayout_2->addWidget(checkBox_clear_marker);

        button_footstep_plan = new QPushButton(groupBox_2);
        button_footstep_plan->setObjectName(QStringLiteral("button_footstep_plan"));

        horizontalLayout_2->addWidget(button_footstep_plan);

        button_footstep_clear = new QPushButton(groupBox_2);
        button_footstep_clear->setObjectName(QStringLiteral("button_footstep_clear"));

        horizontalLayout_2->addWidget(button_footstep_clear);

        button_footstep_go = new QPushButton(groupBox_2);
        button_footstep_go->setObjectName(QStringLiteral("button_footstep_go"));

        horizontalLayout_2->addWidget(button_footstep_go);


        gridLayout->addWidget(groupBox_2, 3, 0, 1, 4);

        groupBox = new QGroupBox(PreviewWalkingForm);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMinimumSize(QSize(0, 420));
        gridLayout_19 = new QGridLayout(groupBox);
        gridLayout_19->setObjectName(QStringLiteral("gridLayout_19"));
        label_41 = new QLabel(groupBox);
        label_41->setObjectName(QStringLiteral("label_41"));

        gridLayout_19->addWidget(label_41, 4, 0, 1, 1);

        button_p_walking_right = new QPushButton(groupBox);
        button_p_walking_right->setObjectName(QStringLiteral("button_p_walking_right"));
        button_p_walking_right->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_right, 7, 2, 1, 1);

        button_p_walking_forward = new QPushButton(groupBox);
        button_p_walking_forward->setObjectName(QStringLiteral("button_p_walking_forward"));
        button_p_walking_forward->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_forward, 6, 1, 1, 1);

        label_38 = new QLabel(groupBox);
        label_38->setObjectName(QStringLiteral("label_38"));

        gridLayout_19->addWidget(label_38, 1, 0, 1, 1);

        button_p_walking_left = new QPushButton(groupBox);
        button_p_walking_left->setObjectName(QStringLiteral("button_p_walking_left"));
        button_p_walking_left->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_left, 7, 0, 1, 1);

        label_37 = new QLabel(groupBox);
        label_37->setObjectName(QStringLiteral("label_37"));

        gridLayout_19->addWidget(label_37, 0, 0, 1, 1);

        button_p_walking_backward = new QPushButton(groupBox);
        button_p_walking_backward->setObjectName(QStringLiteral("button_p_walking_backward"));
        button_p_walking_backward->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_backward, 8, 1, 1, 1);

        dSpinBox_p_walking_step_angle = new QDoubleSpinBox(groupBox);
        dSpinBox_p_walking_step_angle->setObjectName(QStringLiteral("dSpinBox_p_walking_step_angle"));
        dSpinBox_p_walking_step_angle->setMaximum(99);
        dSpinBox_p_walking_step_angle->setSingleStep(1);

        gridLayout_19->addWidget(dSpinBox_p_walking_step_angle, 4, 1, 1, 2);

        button_p_walking_stop = new QPushButton(groupBox);
        button_p_walking_stop->setObjectName(QStringLiteral("button_p_walking_stop"));
        button_p_walking_stop->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_stop, 7, 1, 1, 1);

        label_39 = new QLabel(groupBox);
        label_39->setObjectName(QStringLiteral("label_39"));

        gridLayout_19->addWidget(label_39, 2, 0, 1, 1);

        label_40 = new QLabel(groupBox);
        label_40->setObjectName(QStringLiteral("label_40"));

        gridLayout_19->addWidget(label_40, 3, 0, 1, 1);

        dSpinBox_p_walking_step_length = new QDoubleSpinBox(groupBox);
        dSpinBox_p_walking_step_length->setObjectName(QStringLiteral("dSpinBox_p_walking_step_length"));
        dSpinBox_p_walking_step_length->setMaximum(99);
        dSpinBox_p_walking_step_length->setSingleStep(0.01);

        gridLayout_19->addWidget(dSpinBox_p_walking_step_length, 2, 1, 1, 2);

        dSpinBox_p_walking_step_num = new QDoubleSpinBox(groupBox);
        dSpinBox_p_walking_step_num->setObjectName(QStringLiteral("dSpinBox_p_walking_step_num"));
        dSpinBox_p_walking_step_num->setValue(4);

        gridLayout_19->addWidget(dSpinBox_p_walking_step_num, 1, 1, 1, 2);

        button_p_walking_turn_l = new QPushButton(groupBox);
        button_p_walking_turn_l->setObjectName(QStringLiteral("button_p_walking_turn_l"));
        button_p_walking_turn_l->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_turn_l, 6, 0, 1, 1);

        dSpinBox_p_walking_side_length = new QDoubleSpinBox(groupBox);
        dSpinBox_p_walking_side_length->setObjectName(QStringLiteral("dSpinBox_p_walking_side_length"));
        dSpinBox_p_walking_side_length->setMaximum(99);
        dSpinBox_p_walking_side_length->setSingleStep(0.01);

        gridLayout_19->addWidget(dSpinBox_p_walking_side_length, 3, 1, 1, 2);

        dSpinBox_p_walking_step_time = new QDoubleSpinBox(groupBox);
        dSpinBox_p_walking_step_time->setObjectName(QStringLiteral("dSpinBox_p_walking_step_time"));
        dSpinBox_p_walking_step_time->setSingleStep(0.1);
        dSpinBox_p_walking_step_time->setValue(0.5);

        gridLayout_19->addWidget(dSpinBox_p_walking_step_time, 0, 1, 1, 2);

        button_p_walking_turn_r = new QPushButton(groupBox);
        button_p_walking_turn_r->setObjectName(QStringLiteral("button_p_walking_turn_r"));
        button_p_walking_turn_r->setMinimumSize(QSize(100, 60));

        gridLayout_19->addWidget(button_p_walking_turn_r, 6, 2, 1, 1);

        label_42 = new QLabel(groupBox);
        label_42->setObjectName(QStringLiteral("label_42"));

        gridLayout_19->addWidget(label_42, 5, 0, 1, 1);

        comboBox_p_walking_start_leg = new QComboBox(groupBox);
        comboBox_p_walking_start_leg->setObjectName(QStringLiteral("comboBox_p_walking_start_leg"));

        gridLayout_19->addWidget(comboBox_p_walking_start_leg, 5, 1, 1, 2);


        gridLayout->addWidget(groupBox, 1, 0, 1, 2);

        frame_2 = new QFrame(PreviewWalkingForm);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(50);
        sizePolicy.setVerticalStretch(50);
        sizePolicy.setHeightForWidth(frame_2->sizePolicy().hasHeightForWidth());
        frame_2->setSizePolicy(sizePolicy);
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(frame_2);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        button_p_walking_init_pose = new QPushButton(frame_2);
        button_p_walking_init_pose->setObjectName(QStringLiteral("button_p_walking_init_pose"));

        horizontalLayout->addWidget(button_p_walking_init_pose);

        button_p_walking_balance_on = new QPushButton(frame_2);
        button_p_walking_balance_on->setObjectName(QStringLiteral("button_p_walking_balance_on"));

        horizontalLayout->addWidget(button_p_walking_balance_on);

        button_p_walking_balance_off = new QPushButton(frame_2);
        button_p_walking_balance_off->setObjectName(QStringLiteral("button_p_walking_balance_off"));

        horizontalLayout->addWidget(button_p_walking_balance_off);


        gridLayout->addWidget(frame_2, 0, 0, 1, 4);


        retranslateUi(PreviewWalkingForm);

        QMetaObject::connectSlotsByName(PreviewWalkingForm);
    } // setupUi

    void retranslateUi(QWidget *PreviewWalkingForm)
    {
        PreviewWalkingForm->setWindowTitle(QApplication::translate("PreviewWalkingForm", "Form", 0));
        groupBox_6->setTitle(QApplication::translate("PreviewWalkingForm", "Interactive Marker", 0));
        dSpinBox_marker_pos_x->setSuffix(QApplication::translate("PreviewWalkingForm", " m", 0));
        label_16->setText(QApplication::translate("PreviewWalkingForm", "z", 0));
        label_14->setText(QApplication::translate("PreviewWalkingForm", "x", 0));
        dSpinBox_marker_ori_p->setSuffix(QApplication::translate("PreviewWalkingForm", " \313\232", 0));
        button_marker_clear->setText(QApplication::translate("PreviewWalkingForm", "Clear", 0));
        label_19->setText(QApplication::translate("PreviewWalkingForm", "y", 0));
        dSpinBox_marker_ori_y->setSuffix(QApplication::translate("PreviewWalkingForm", " \313\232", 0));
        dSpinBox_marker_pos_y->setSuffix(QApplication::translate("PreviewWalkingForm", " m", 0));
        label_17->setText(QApplication::translate("PreviewWalkingForm", "r", 0));
        dSpinBox_marker_pos_z->setSuffix(QApplication::translate("PreviewWalkingForm", " m", 0));
        label_18->setText(QApplication::translate("PreviewWalkingForm", "p", 0));
        label_15->setText(QApplication::translate("PreviewWalkingForm", "y", 0));
        button_marker_set->setText(QApplication::translate("PreviewWalkingForm", "Set", 0));
        dSpinBox_marker_ori_r->setSuffix(QApplication::translate("PreviewWalkingForm", " \313\232", 0));
        walkin_param->setTitle(QApplication::translate("PreviewWalkingForm", "Walking Param", 0));
        label_46->setText(QApplication::translate("PreviewWalkingForm", "ZMP Offset x [m]", 0));
        label_43->setText(QApplication::translate("PreviewWalkingForm", "DSP Ratio", 0));
        button_set_walking_param->setText(QApplication::translate("PreviewWalkingForm", "Set Param", 0));
        button_send_body_offset->setText(QApplication::translate("PreviewWalkingForm", "Set Body Offset", 0));
        label_44->setText(QApplication::translate("PreviewWalkingForm", "LIPM Height [m]", 0));
        label_47->setText(QApplication::translate("PreviewWalkingForm", "ZMP Offset y [m]", 0));
        label_45->setText(QApplication::translate("PreviewWalkingForm", "Foot Height Max [m]", 0));
        label_64->setText(QApplication::translate("PreviewWalkingForm", "Body Offset y [m]", 0));
        label_66->setText(QApplication::translate("PreviewWalkingForm", "Foot Distance [m]", 0));
        label_63->setText(QApplication::translate("PreviewWalkingForm", "Body Offset x [m]", 0));
        label_65->setText(QApplication::translate("PreviewWalkingForm", "Body Offset z [m]", 0));
        button_send_foot_distance->setText(QApplication::translate("PreviewWalkingForm", "Set Foot Distance", 0));
        groupBox_2->setTitle(QApplication::translate("PreviewWalkingForm", "Footstep Planer", 0));
        checkBox_clear_path->setText(QApplication::translate("PreviewWalkingForm", "Clear Path", 0));
        checkBox_clear_marker->setText(QApplication::translate("PreviewWalkingForm", "Clear Marker", 0));
        button_footstep_plan->setText(QApplication::translate("PreviewWalkingForm", "Plan", 0));
        button_footstep_clear->setText(QApplication::translate("PreviewWalkingForm", "Clear", 0));
        button_footstep_go->setText(QApplication::translate("PreviewWalkingForm", "Go", 0));
        groupBox->setTitle(QApplication::translate("PreviewWalkingForm", "Goal State", 0));
        label_41->setText(QApplication::translate("PreviewWalkingForm", "Step Angle", 0));
        button_p_walking_right->setText(QApplication::translate("PreviewWalkingForm", "Right", 0));
        button_p_walking_forward->setText(QApplication::translate("PreviewWalkingForm", "Forward", 0));
        label_38->setText(QApplication::translate("PreviewWalkingForm", "Step Number", 0));
        button_p_walking_left->setText(QApplication::translate("PreviewWalkingForm", "Left", 0));
        label_37->setText(QApplication::translate("PreviewWalkingForm", "Step Time", 0));
        button_p_walking_backward->setText(QApplication::translate("PreviewWalkingForm", "Backward", 0));
        button_p_walking_stop->setText(QApplication::translate("PreviewWalkingForm", "Stop", 0));
        label_39->setText(QApplication::translate("PreviewWalkingForm", "Step Length", 0));
        label_40->setText(QApplication::translate("PreviewWalkingForm", "Side Length", 0));
        button_p_walking_turn_l->setText(QApplication::translate("PreviewWalkingForm", "Turn Left", 0));
        button_p_walking_turn_r->setText(QApplication::translate("PreviewWalkingForm", "Turn Right", 0));
        label_42->setText(QApplication::translate("PreviewWalkingForm", "Start Leg", 0));
        comboBox_p_walking_start_leg->clear();
        comboBox_p_walking_start_leg->insertItems(0, QStringList()
         << QApplication::translate("PreviewWalkingForm", "right_leg", 0)
         << QApplication::translate("PreviewWalkingForm", "left_leg", 0)
        );
        button_p_walking_init_pose->setText(QApplication::translate("PreviewWalkingForm", "Go to Initial Pose", 0));
        button_p_walking_balance_on->setText(QApplication::translate("PreviewWalkingForm", "Balance On", 0));
        button_p_walking_balance_off->setText(QApplication::translate("PreviewWalkingForm", "Balance Off", 0));
    } // retranslateUi

};

namespace Ui {
    class PreviewWalkingForm: public Ui_PreviewWalkingForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PREVIEW_WALKING_FORM_H
