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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
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
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *logging_groupBox;
    QGridLayout *gridLayout_3;
    QListView *view_logging;
    QPushButton *clear_button;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QTabWidget *tab;
    QWidget *right_arm_tab;
    QVBoxLayout *verticalLayout_4;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_6;
    QLabel *right_arm_modval;
    QGroupBox *right_arm_group;
    QGridLayout *gridLayout;
    QLabel *r_arm_present_pos;
    QLabel *r_arm_modval;
    QLabel *r_arm_i_gain;
    QLabel *label;
    QLabel *r_arm_p_gain;
    QLabel *r_arm_goal_pos;
    QLabel *label_2;
    QLabel *r_arm_deg_4;
    QLabel *r_arm_deg_3;
    QLabel *r_arm_d_gain;
    QLabel *r_arm_offset;
    QLabel *r_arm_deg_2;
    QLabel *r_arm_deg_1;
    QGroupBox *right_arm_torque;
    QGridLayout *gridLayout_2;
    QSpacerItem *verticalSpacer;
    QWidget *left_arm_tab;
    QVBoxLayout *verticalLayout_5;
    QScrollArea *scrollArea_2;
    QWidget *scrollAreaWidgetContents_2;
    QVBoxLayout *verticalLayout_7;
    QLabel *left_arm_modval;
    QGroupBox *left_arm_group;
    QGridLayout *gridLayout_4;
    QLabel *l_arm_i_gain;
    QLabel *l_arm_present_pos;
    QLabel *l_arm_goal_pos;
    QLabel *l_arm_modval;
    QLabel *l_arm_p_gain;
    QLabel *l_arm_deg_1;
    QLabel *l_arm_offset;
    QLabel *l_arm_deg_4;
    QLabel *l_arm_deg_3;
    QLabel *l_arm_deg_2;
    QLabel *l_arm_d_gain;
    QLabel *label_4;
    QLabel *label_3;
    QGroupBox *left_arm_torque;
    QGridLayout *gridLayout_5;
    QSpacerItem *verticalSpacer_2;
    QWidget *leg_tab;
    QGridLayout *gridLayout_6;
    QScrollArea *scrollArea_3;
    QWidget *scrollAreaWidgetContents_3;
    QVBoxLayout *verticalLayout_8;
    QLabel *leg_modval;
    QGroupBox *leg_group;
    QGridLayout *gridLayout_7;
    QLabel *leg_deg_4;
    QLabel *leg_modval_2;
    QLabel *leg_offset;
    QLabel *leg_deg_3;
    QLabel *leg_p_gain;
    QLabel *leg_deg_2;
    QLabel *leg_i_gain;
    QLabel *leg_deg_1;
    QLabel *leg_present_pos;
    QLabel *leg_goal_pos;
    QLabel *leg_d_gain;
    QLabel *label_6;
    QLabel *label_5;
    QGroupBox *leg_torque;
    QGridLayout *gridLayout_8;
    QSpacerItem *verticalSpacer_3;
    QWidget *body_tab;
    QGridLayout *gridLayout_9;
    QScrollArea *scrollArea_4;
    QWidget *scrollAreaWidgetContents_4;
    QVBoxLayout *verticalLayout_9;
    QLabel *body_modval;
    QGroupBox *body_group;
    QGridLayout *gridLayout_10;
    QLabel *body_i_gain;
    QLabel *body_p_gain;
    QLabel *body_preset_pos;
    QLabel *body_deg_3;
    QLabel *body_modval_2;
    QLabel *body_deg_2;
    QLabel *body_d_gain;
    QLabel *body_offset;
    QLabel *body_goal_pos;
    QLabel *body_deg_1;
    QLabel *body_deg_4;
    QLabel *label_7;
    QLabel *label_8;
    QGroupBox *body_torque;
    QGridLayout *gridLayout_11;
    QLabel *empty_1;
    QLabel *empty_3;
    QLabel *empty_2;
    QSpacerItem *verticalSpacer_4;
    QWidget *widget;
    QGridLayout *gridLayout_12;
    QPushButton *inipose_button;
    QPushButton *refresh_button;
    QPushButton *save_offset_button;
    QPushButton *save_gain_button;
    QPushButton *tuning_pose_button;
    QComboBox *tuning_pose_comboBox;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QStringLiteral("MainWindowDesign"));
        MainWindowDesign->resize(1644, 974);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindowDesign->sizePolicy().hasHeightForWidth());
        MainWindowDesign->setSizePolicy(sizePolicy);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/robotis_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
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
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tab_manager->sizePolicy().hasHeightForWidth());
        tab_manager->setSizePolicy(sizePolicy1);
        tab_manager->setMinimumSize(QSize(500, 0));
        tab_manager->setMaximumSize(QSize(16777215, 1000));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QStringLiteral("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        logging_groupBox = new QGroupBox(tab_status);
        logging_groupBox->setObjectName(QStringLiteral("logging_groupBox"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(logging_groupBox->sizePolicy().hasHeightForWidth());
        logging_groupBox->setSizePolicy(sizePolicy2);
        gridLayout_3 = new QGridLayout(logging_groupBox);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        view_logging = new QListView(logging_groupBox);
        view_logging->setObjectName(QStringLiteral("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);

        clear_button = new QPushButton(logging_groupBox);
        clear_button->setObjectName(QStringLiteral("clear_button"));

        gridLayout_3->addWidget(clear_button, 1, 0, 1, 1);


        verticalLayout_2->addWidget(logging_groupBox);

        tab_manager->addTab(tab_status, QString());

        horizontalLayout_3->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1644, 19));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        dock_status = new QDockWidget(MainWindowDesign);
        dock_status->setObjectName(QStringLiteral("dock_status"));
        sizePolicy1.setHeightForWidth(dock_status->sizePolicy().hasHeightForWidth());
        dock_status->setSizePolicy(sizePolicy1);
        dock_status->setMinimumSize(QSize(1100, 878));
        dock_status->setAllowedAreas(Qt::RightDockWidgetArea);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QStringLiteral("dockWidgetContents_2"));
        sizePolicy1.setHeightForWidth(dockWidgetContents_2->sizePolicy().hasHeightForWidth());
        dockWidgetContents_2->setSizePolicy(sizePolicy1);
        verticalLayout = new QVBoxLayout(dockWidgetContents_2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        frame = new QFrame(dockWidgetContents_2);
        frame->setObjectName(QStringLiteral("frame"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy3);
        frame->setMaximumSize(QSize(16777215, 1000));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        tab = new QTabWidget(frame);
        tab->setObjectName(QStringLiteral("tab"));
        sizePolicy3.setHeightForWidth(tab->sizePolicy().hasHeightForWidth());
        tab->setSizePolicy(sizePolicy3);
        tab->setMinimumSize(QSize(0, 0));
        tab->setMaximumSize(QSize(16777215, 3000));
        right_arm_tab = new QWidget();
        right_arm_tab->setObjectName(QStringLiteral("right_arm_tab"));
        verticalLayout_4 = new QVBoxLayout(right_arm_tab);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        scrollArea = new QScrollArea(right_arm_tab);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 1038, 742));
        verticalLayout_6 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        right_arm_modval = new QLabel(scrollAreaWidgetContents);
        right_arm_modval->setObjectName(QStringLiteral("right_arm_modval"));
        right_arm_modval->setMaximumSize(QSize(16777215, 20));
        right_arm_modval->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_6->addWidget(right_arm_modval);

        right_arm_group = new QGroupBox(scrollAreaWidgetContents);
        right_arm_group->setObjectName(QStringLiteral("right_arm_group"));
        sizePolicy3.setHeightForWidth(right_arm_group->sizePolicy().hasHeightForWidth());
        right_arm_group->setSizePolicy(sizePolicy3);
        right_arm_group->setMaximumSize(QSize(16777215, 350));
        gridLayout = new QGridLayout(right_arm_group);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        r_arm_present_pos = new QLabel(right_arm_group);
        r_arm_present_pos->setObjectName(QStringLiteral("r_arm_present_pos"));

        gridLayout->addWidget(r_arm_present_pos, 1, 4, 1, 1, Qt::AlignHCenter);

        r_arm_modval = new QLabel(right_arm_group);
        r_arm_modval->setObjectName(QStringLiteral("r_arm_modval"));

        gridLayout->addWidget(r_arm_modval, 1, 3, 1, 1, Qt::AlignHCenter);

        r_arm_i_gain = new QLabel(right_arm_group);
        r_arm_i_gain->setObjectName(QStringLiteral("r_arm_i_gain"));

        gridLayout->addWidget(r_arm_i_gain, 1, 6, 1, 1, Qt::AlignHCenter);

        label = new QLabel(right_arm_group);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        r_arm_p_gain = new QLabel(right_arm_group);
        r_arm_p_gain->setObjectName(QStringLiteral("r_arm_p_gain"));

        gridLayout->addWidget(r_arm_p_gain, 1, 5, 1, 1, Qt::AlignHCenter);

        r_arm_goal_pos = new QLabel(right_arm_group);
        r_arm_goal_pos->setObjectName(QStringLiteral("r_arm_goal_pos"));

        gridLayout->addWidget(r_arm_goal_pos, 1, 1, 1, 1, Qt::AlignHCenter);

        label_2 = new QLabel(right_arm_group);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        r_arm_deg_4 = new QLabel(right_arm_group);
        r_arm_deg_4->setObjectName(QStringLiteral("r_arm_deg_4"));

        gridLayout->addWidget(r_arm_deg_4, 2, 4, 1, 1, Qt::AlignHCenter);

        r_arm_deg_3 = new QLabel(right_arm_group);
        r_arm_deg_3->setObjectName(QStringLiteral("r_arm_deg_3"));

        gridLayout->addWidget(r_arm_deg_3, 2, 3, 1, 1, Qt::AlignHCenter);

        r_arm_d_gain = new QLabel(right_arm_group);
        r_arm_d_gain->setObjectName(QStringLiteral("r_arm_d_gain"));

        gridLayout->addWidget(r_arm_d_gain, 1, 7, 1, 1, Qt::AlignHCenter);

        r_arm_offset = new QLabel(right_arm_group);
        r_arm_offset->setObjectName(QStringLiteral("r_arm_offset"));

        gridLayout->addWidget(r_arm_offset, 1, 2, 1, 1, Qt::AlignHCenter);

        r_arm_deg_2 = new QLabel(right_arm_group);
        r_arm_deg_2->setObjectName(QStringLiteral("r_arm_deg_2"));

        gridLayout->addWidget(r_arm_deg_2, 2, 2, 1, 1, Qt::AlignHCenter);

        r_arm_deg_1 = new QLabel(right_arm_group);
        r_arm_deg_1->setObjectName(QStringLiteral("r_arm_deg_1"));

        gridLayout->addWidget(r_arm_deg_1, 2, 1, 1, 1, Qt::AlignHCenter|Qt::AlignBottom);


        verticalLayout_6->addWidget(right_arm_group);

        right_arm_torque = new QGroupBox(scrollAreaWidgetContents);
        right_arm_torque->setObjectName(QStringLiteral("right_arm_torque"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(right_arm_torque->sizePolicy().hasHeightForWidth());
        right_arm_torque->setSizePolicy(sizePolicy4);
        right_arm_torque->setMaximumSize(QSize(2100, 200));
        gridLayout_2 = new QGridLayout(right_arm_torque);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));

        verticalLayout_6->addWidget(right_arm_torque);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_4->addWidget(scrollArea);

        tab->addTab(right_arm_tab, QString());
        left_arm_tab = new QWidget();
        left_arm_tab->setObjectName(QStringLiteral("left_arm_tab"));
        verticalLayout_5 = new QVBoxLayout(left_arm_tab);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        scrollArea_2 = new QScrollArea(left_arm_tab);
        scrollArea_2->setObjectName(QStringLiteral("scrollArea_2"));
        scrollArea_2->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QStringLiteral("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, 0, 417, 166));
        verticalLayout_7 = new QVBoxLayout(scrollAreaWidgetContents_2);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        left_arm_modval = new QLabel(scrollAreaWidgetContents_2);
        left_arm_modval->setObjectName(QStringLiteral("left_arm_modval"));
        left_arm_modval->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_7->addWidget(left_arm_modval);

        left_arm_group = new QGroupBox(scrollAreaWidgetContents_2);
        left_arm_group->setObjectName(QStringLiteral("left_arm_group"));
        sizePolicy3.setHeightForWidth(left_arm_group->sizePolicy().hasHeightForWidth());
        left_arm_group->setSizePolicy(sizePolicy3);
        left_arm_group->setMaximumSize(QSize(16777215, 350));
        gridLayout_4 = new QGridLayout(left_arm_group);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        l_arm_i_gain = new QLabel(left_arm_group);
        l_arm_i_gain->setObjectName(QStringLiteral("l_arm_i_gain"));

        gridLayout_4->addWidget(l_arm_i_gain, 0, 6, 1, 1, Qt::AlignHCenter);

        l_arm_present_pos = new QLabel(left_arm_group);
        l_arm_present_pos->setObjectName(QStringLiteral("l_arm_present_pos"));

        gridLayout_4->addWidget(l_arm_present_pos, 0, 4, 1, 1, Qt::AlignHCenter);

        l_arm_goal_pos = new QLabel(left_arm_group);
        l_arm_goal_pos->setObjectName(QStringLiteral("l_arm_goal_pos"));

        gridLayout_4->addWidget(l_arm_goal_pos, 0, 1, 1, 1, Qt::AlignHCenter);

        l_arm_modval = new QLabel(left_arm_group);
        l_arm_modval->setObjectName(QStringLiteral("l_arm_modval"));

        gridLayout_4->addWidget(l_arm_modval, 0, 3, 1, 1, Qt::AlignHCenter);

        l_arm_p_gain = new QLabel(left_arm_group);
        l_arm_p_gain->setObjectName(QStringLiteral("l_arm_p_gain"));

        gridLayout_4->addWidget(l_arm_p_gain, 0, 5, 1, 1, Qt::AlignHCenter);

        l_arm_deg_1 = new QLabel(left_arm_group);
        l_arm_deg_1->setObjectName(QStringLiteral("l_arm_deg_1"));

        gridLayout_4->addWidget(l_arm_deg_1, 1, 1, 1, 1, Qt::AlignHCenter);

        l_arm_offset = new QLabel(left_arm_group);
        l_arm_offset->setObjectName(QStringLiteral("l_arm_offset"));

        gridLayout_4->addWidget(l_arm_offset, 0, 2, 1, 1, Qt::AlignHCenter);

        l_arm_deg_4 = new QLabel(left_arm_group);
        l_arm_deg_4->setObjectName(QStringLiteral("l_arm_deg_4"));

        gridLayout_4->addWidget(l_arm_deg_4, 1, 4, 1, 1, Qt::AlignHCenter);

        l_arm_deg_3 = new QLabel(left_arm_group);
        l_arm_deg_3->setObjectName(QStringLiteral("l_arm_deg_3"));

        gridLayout_4->addWidget(l_arm_deg_3, 1, 3, 1, 1, Qt::AlignHCenter);

        l_arm_deg_2 = new QLabel(left_arm_group);
        l_arm_deg_2->setObjectName(QStringLiteral("l_arm_deg_2"));

        gridLayout_4->addWidget(l_arm_deg_2, 1, 2, 1, 1, Qt::AlignHCenter);

        l_arm_d_gain = new QLabel(left_arm_group);
        l_arm_d_gain->setObjectName(QStringLiteral("l_arm_d_gain"));

        gridLayout_4->addWidget(l_arm_d_gain, 0, 7, 1, 1, Qt::AlignHCenter);

        label_4 = new QLabel(left_arm_group);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_4->addWidget(label_4, 1, 0, 1, 1);

        label_3 = new QLabel(left_arm_group);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_4->addWidget(label_3, 0, 0, 1, 1);


        verticalLayout_7->addWidget(left_arm_group);

        left_arm_torque = new QGroupBox(scrollAreaWidgetContents_2);
        left_arm_torque->setObjectName(QStringLiteral("left_arm_torque"));
        left_arm_torque->setMaximumSize(QSize(2000, 150));
        gridLayout_5 = new QGridLayout(left_arm_torque);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));

        verticalLayout_7->addWidget(left_arm_torque);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer_2);

        scrollArea_2->setWidget(scrollAreaWidgetContents_2);

        verticalLayout_5->addWidget(scrollArea_2);

        tab->addTab(left_arm_tab, QString());
        leg_tab = new QWidget();
        leg_tab->setObjectName(QStringLiteral("leg_tab"));
        gridLayout_6 = new QGridLayout(leg_tab);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        scrollArea_3 = new QScrollArea(leg_tab);
        scrollArea_3->setObjectName(QStringLiteral("scrollArea_3"));
        scrollArea_3->setWidgetResizable(true);
        scrollAreaWidgetContents_3 = new QWidget();
        scrollAreaWidgetContents_3->setObjectName(QStringLiteral("scrollAreaWidgetContents_3"));
        scrollAreaWidgetContents_3->setGeometry(QRect(0, 0, 416, 166));
        verticalLayout_8 = new QVBoxLayout(scrollAreaWidgetContents_3);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        leg_modval = new QLabel(scrollAreaWidgetContents_3);
        leg_modval->setObjectName(QStringLiteral("leg_modval"));
        leg_modval->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_8->addWidget(leg_modval);

        leg_group = new QGroupBox(scrollAreaWidgetContents_3);
        leg_group->setObjectName(QStringLiteral("leg_group"));
        gridLayout_7 = new QGridLayout(leg_group);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        leg_deg_4 = new QLabel(leg_group);
        leg_deg_4->setObjectName(QStringLiteral("leg_deg_4"));

        gridLayout_7->addWidget(leg_deg_4, 1, 4, 1, 1, Qt::AlignHCenter);

        leg_modval_2 = new QLabel(leg_group);
        leg_modval_2->setObjectName(QStringLiteral("leg_modval_2"));

        gridLayout_7->addWidget(leg_modval_2, 0, 3, 1, 1, Qt::AlignHCenter);

        leg_offset = new QLabel(leg_group);
        leg_offset->setObjectName(QStringLiteral("leg_offset"));

        gridLayout_7->addWidget(leg_offset, 0, 2, 1, 1, Qt::AlignHCenter);

        leg_deg_3 = new QLabel(leg_group);
        leg_deg_3->setObjectName(QStringLiteral("leg_deg_3"));

        gridLayout_7->addWidget(leg_deg_3, 1, 3, 1, 1, Qt::AlignHCenter);

        leg_p_gain = new QLabel(leg_group);
        leg_p_gain->setObjectName(QStringLiteral("leg_p_gain"));

        gridLayout_7->addWidget(leg_p_gain, 0, 5, 1, 1, Qt::AlignHCenter);

        leg_deg_2 = new QLabel(leg_group);
        leg_deg_2->setObjectName(QStringLiteral("leg_deg_2"));

        gridLayout_7->addWidget(leg_deg_2, 1, 2, 1, 1, Qt::AlignHCenter);

        leg_i_gain = new QLabel(leg_group);
        leg_i_gain->setObjectName(QStringLiteral("leg_i_gain"));

        gridLayout_7->addWidget(leg_i_gain, 0, 6, 1, 1, Qt::AlignHCenter);

        leg_deg_1 = new QLabel(leg_group);
        leg_deg_1->setObjectName(QStringLiteral("leg_deg_1"));

        gridLayout_7->addWidget(leg_deg_1, 1, 1, 1, 1, Qt::AlignHCenter);

        leg_present_pos = new QLabel(leg_group);
        leg_present_pos->setObjectName(QStringLiteral("leg_present_pos"));

        gridLayout_7->addWidget(leg_present_pos, 0, 4, 1, 1, Qt::AlignHCenter);

        leg_goal_pos = new QLabel(leg_group);
        leg_goal_pos->setObjectName(QStringLiteral("leg_goal_pos"));

        gridLayout_7->addWidget(leg_goal_pos, 0, 1, 1, 1, Qt::AlignHCenter);

        leg_d_gain = new QLabel(leg_group);
        leg_d_gain->setObjectName(QStringLiteral("leg_d_gain"));

        gridLayout_7->addWidget(leg_d_gain, 0, 7, 1, 1, Qt::AlignHCenter);

        label_6 = new QLabel(leg_group);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_7->addWidget(label_6, 1, 0, 1, 1);

        label_5 = new QLabel(leg_group);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_7->addWidget(label_5, 0, 0, 1, 1);


        verticalLayout_8->addWidget(leg_group);

        leg_torque = new QGroupBox(scrollAreaWidgetContents_3);
        leg_torque->setObjectName(QStringLiteral("leg_torque"));
        gridLayout_8 = new QGridLayout(leg_torque);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));

        verticalLayout_8->addWidget(leg_torque);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_3);

        scrollArea_3->setWidget(scrollAreaWidgetContents_3);

        gridLayout_6->addWidget(scrollArea_3, 0, 0, 1, 1);

        tab->addTab(leg_tab, QString());
        body_tab = new QWidget();
        body_tab->setObjectName(QStringLiteral("body_tab"));
        gridLayout_9 = new QGridLayout(body_tab);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        scrollArea_4 = new QScrollArea(body_tab);
        scrollArea_4->setObjectName(QStringLiteral("scrollArea_4"));
        scrollArea_4->setWidgetResizable(true);
        scrollAreaWidgetContents_4 = new QWidget();
        scrollAreaWidgetContents_4->setObjectName(QStringLiteral("scrollAreaWidgetContents_4"));
        scrollAreaWidgetContents_4->setGeometry(QRect(0, 0, 416, 180));
        verticalLayout_9 = new QVBoxLayout(scrollAreaWidgetContents_4);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        body_modval = new QLabel(scrollAreaWidgetContents_4);
        body_modval->setObjectName(QStringLiteral("body_modval"));
        body_modval->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_9->addWidget(body_modval);

        body_group = new QGroupBox(scrollAreaWidgetContents_4);
        body_group->setObjectName(QStringLiteral("body_group"));
        gridLayout_10 = new QGridLayout(body_group);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        body_i_gain = new QLabel(body_group);
        body_i_gain->setObjectName(QStringLiteral("body_i_gain"));

        gridLayout_10->addWidget(body_i_gain, 0, 6, 1, 1, Qt::AlignHCenter);

        body_p_gain = new QLabel(body_group);
        body_p_gain->setObjectName(QStringLiteral("body_p_gain"));

        gridLayout_10->addWidget(body_p_gain, 0, 5, 1, 1, Qt::AlignHCenter);

        body_preset_pos = new QLabel(body_group);
        body_preset_pos->setObjectName(QStringLiteral("body_preset_pos"));

        gridLayout_10->addWidget(body_preset_pos, 0, 4, 1, 1, Qt::AlignHCenter);

        body_deg_3 = new QLabel(body_group);
        body_deg_3->setObjectName(QStringLiteral("body_deg_3"));

        gridLayout_10->addWidget(body_deg_3, 1, 3, 1, 1, Qt::AlignHCenter);

        body_modval_2 = new QLabel(body_group);
        body_modval_2->setObjectName(QStringLiteral("body_modval_2"));

        gridLayout_10->addWidget(body_modval_2, 0, 3, 1, 1, Qt::AlignHCenter);

        body_deg_2 = new QLabel(body_group);
        body_deg_2->setObjectName(QStringLiteral("body_deg_2"));

        gridLayout_10->addWidget(body_deg_2, 1, 2, 1, 1, Qt::AlignHCenter);

        body_d_gain = new QLabel(body_group);
        body_d_gain->setObjectName(QStringLiteral("body_d_gain"));

        gridLayout_10->addWidget(body_d_gain, 0, 7, 1, 1, Qt::AlignHCenter);

        body_offset = new QLabel(body_group);
        body_offset->setObjectName(QStringLiteral("body_offset"));

        gridLayout_10->addWidget(body_offset, 0, 2, 1, 1, Qt::AlignHCenter);

        body_goal_pos = new QLabel(body_group);
        body_goal_pos->setObjectName(QStringLiteral("body_goal_pos"));

        gridLayout_10->addWidget(body_goal_pos, 0, 1, 1, 1, Qt::AlignHCenter);

        body_deg_1 = new QLabel(body_group);
        body_deg_1->setObjectName(QStringLiteral("body_deg_1"));

        gridLayout_10->addWidget(body_deg_1, 1, 1, 1, 1, Qt::AlignHCenter);

        body_deg_4 = new QLabel(body_group);
        body_deg_4->setObjectName(QStringLiteral("body_deg_4"));

        gridLayout_10->addWidget(body_deg_4, 1, 4, 1, 1, Qt::AlignHCenter);

        label_7 = new QLabel(body_group);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_10->addWidget(label_7, 0, 0, 1, 1);

        label_8 = new QLabel(body_group);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_10->addWidget(label_8, 1, 0, 1, 1);


        verticalLayout_9->addWidget(body_group);

        body_torque = new QGroupBox(scrollAreaWidgetContents_4);
        body_torque->setObjectName(QStringLiteral("body_torque"));
        gridLayout_11 = new QGridLayout(body_torque);
        gridLayout_11->setObjectName(QStringLiteral("gridLayout_11"));
        empty_1 = new QLabel(body_torque);
        empty_1->setObjectName(QStringLiteral("empty_1"));

        gridLayout_11->addWidget(empty_1, 0, 0, 1, 1);

        empty_3 = new QLabel(body_torque);
        empty_3->setObjectName(QStringLiteral("empty_3"));

        gridLayout_11->addWidget(empty_3, 0, 2, 1, 1);

        empty_2 = new QLabel(body_torque);
        empty_2->setObjectName(QStringLiteral("empty_2"));

        gridLayout_11->addWidget(empty_2, 0, 1, 1, 1);


        verticalLayout_9->addWidget(body_torque);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_4);

        scrollArea_4->setWidget(scrollAreaWidgetContents_4);

        gridLayout_9->addWidget(scrollArea_4, 0, 0, 1, 1);

        tab->addTab(body_tab, QString());

        verticalLayout_3->addWidget(tab);


        verticalLayout->addWidget(frame);

        widget = new QWidget(dockWidgetContents_2);
        widget->setObjectName(QStringLiteral("widget"));
        sizePolicy3.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy3);
        widget->setMinimumSize(QSize(0, 80));
        widget->setMaximumSize(QSize(16777215, 50));
        gridLayout_12 = new QGridLayout(widget);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        inipose_button = new QPushButton(widget);
        inipose_button->setObjectName(QStringLiteral("inipose_button"));

        gridLayout_12->addWidget(inipose_button, 0, 1, 1, 1);

        refresh_button = new QPushButton(widget);
        refresh_button->setObjectName(QStringLiteral("refresh_button"));

        gridLayout_12->addWidget(refresh_button, 1, 1, 1, 1);

        save_offset_button = new QPushButton(widget);
        save_offset_button->setObjectName(QStringLiteral("save_offset_button"));

        gridLayout_12->addWidget(save_offset_button, 1, 3, 1, 1);

        save_gain_button = new QPushButton(widget);
        save_gain_button->setObjectName(QStringLiteral("save_gain_button"));

        gridLayout_12->addWidget(save_gain_button, 1, 4, 1, 1);

        tuning_pose_button = new QPushButton(widget);
        tuning_pose_button->setObjectName(QStringLiteral("tuning_pose_button"));

        gridLayout_12->addWidget(tuning_pose_button, 0, 4, 1, 1);

        tuning_pose_comboBox = new QComboBox(widget);
        tuning_pose_comboBox->setObjectName(QStringLiteral("tuning_pose_comboBox"));

        gridLayout_12->addWidget(tuning_pose_comboBox, 0, 3, 1, 1);


        verticalLayout->addWidget(widget);

        dock_status->setWidget(dockWidgetContents_2);
        MainWindowDesign->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dock_status);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);
        tab->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "ROBOTIS-OP3 Tuner Client", 0));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0));
        logging_groupBox->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0));
        clear_button->setText(QApplication::translate("MainWindowDesign", "Clear", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0));
        dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Command Panel", 0));
        right_arm_modval->setText(QApplication::translate("MainWindowDesign", "(Modval = Goal Pos. + Offset) ", 0));
        right_arm_group->setTitle(QApplication::translate("MainWindowDesign", "Joint Offset Data", 0));
        r_arm_present_pos->setText(QApplication::translate("MainWindowDesign", "Present Pos.", 0));
        r_arm_modval->setText(QApplication::translate("MainWindowDesign", "ModVal", 0));
        r_arm_i_gain->setText(QApplication::translate("MainWindowDesign", "I Gain", 0));
        label->setText(QString());
        r_arm_p_gain->setText(QApplication::translate("MainWindowDesign", "P Gain", 0));
        r_arm_goal_pos->setText(QApplication::translate("MainWindowDesign", "Goal Pos. ", 0));
        label_2->setText(QString());
        r_arm_deg_4->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        r_arm_deg_3->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        r_arm_d_gain->setText(QApplication::translate("MainWindowDesign", "D Gain", 0));
        r_arm_offset->setText(QApplication::translate("MainWindowDesign", "Offset ", 0));
        r_arm_deg_2->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        r_arm_deg_1->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        right_arm_torque->setTitle(QApplication::translate("MainWindowDesign", "Torque On/Off", 0));
        tab->setTabText(tab->indexOf(right_arm_tab), QApplication::translate("MainWindowDesign", "Right Arm", 0));
        left_arm_modval->setText(QApplication::translate("MainWindowDesign", "(Modval = Goal Pos. + Offset) ", 0));
        left_arm_group->setTitle(QApplication::translate("MainWindowDesign", "Joint Offset Data", 0));
        l_arm_i_gain->setText(QApplication::translate("MainWindowDesign", "I Gain", 0));
        l_arm_present_pos->setText(QApplication::translate("MainWindowDesign", "Present Pos.", 0));
        l_arm_goal_pos->setText(QApplication::translate("MainWindowDesign", "Goal Pos.", 0));
        l_arm_modval->setText(QApplication::translate("MainWindowDesign", "ModVal", 0));
        l_arm_p_gain->setText(QApplication::translate("MainWindowDesign", "P Gain", 0));
        l_arm_deg_1->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        l_arm_offset->setText(QApplication::translate("MainWindowDesign", "Offset", 0));
        l_arm_deg_4->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        l_arm_deg_3->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        l_arm_deg_2->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        l_arm_d_gain->setText(QApplication::translate("MainWindowDesign", "D Gain", 0));
        label_4->setText(QString());
        label_3->setText(QString());
        left_arm_torque->setTitle(QApplication::translate("MainWindowDesign", "Torque On/Off", 0));
        tab->setTabText(tab->indexOf(left_arm_tab), QApplication::translate("MainWindowDesign", "Left Arm", 0));
        leg_modval->setText(QApplication::translate("MainWindowDesign", "(Modval = Goal Pos. + Offset) ", 0));
        leg_group->setTitle(QApplication::translate("MainWindowDesign", "Joint Offset Data", 0));
        leg_deg_4->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        leg_modval_2->setText(QApplication::translate("MainWindowDesign", "Modval", 0));
        leg_offset->setText(QApplication::translate("MainWindowDesign", "Offset", 0));
        leg_deg_3->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        leg_p_gain->setText(QApplication::translate("MainWindowDesign", "P Gain", 0));
        leg_deg_2->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        leg_i_gain->setText(QApplication::translate("MainWindowDesign", "I Gain", 0));
        leg_deg_1->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        leg_present_pos->setText(QApplication::translate("MainWindowDesign", "Present Pos.", 0));
        leg_goal_pos->setText(QApplication::translate("MainWindowDesign", "Goal Pos.", 0));
        leg_d_gain->setText(QApplication::translate("MainWindowDesign", "D Gain", 0));
        label_6->setText(QString());
        label_5->setText(QString());
        leg_torque->setTitle(QApplication::translate("MainWindowDesign", "Torque On/Off", 0));
        tab->setTabText(tab->indexOf(leg_tab), QApplication::translate("MainWindowDesign", "Legs", 0));
        body_modval->setText(QApplication::translate("MainWindowDesign", "(Modval = Goal Pos. + Offset) ", 0));
        body_group->setTitle(QApplication::translate("MainWindowDesign", "Joint Offset Data", 0));
        body_i_gain->setText(QApplication::translate("MainWindowDesign", "I Gain", 0));
        body_p_gain->setText(QApplication::translate("MainWindowDesign", "P Gain", 0));
        body_preset_pos->setText(QApplication::translate("MainWindowDesign", "Present Pos.", 0));
        body_deg_3->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        body_modval_2->setText(QApplication::translate("MainWindowDesign", "Modval", 0));
        body_deg_2->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        body_d_gain->setText(QApplication::translate("MainWindowDesign", "D Gain", 0));
        body_offset->setText(QApplication::translate("MainWindowDesign", "Offset", 0));
        body_goal_pos->setText(QApplication::translate("MainWindowDesign", "Goal Pos.", 0));
        body_deg_1->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        body_deg_4->setText(QApplication::translate("MainWindowDesign", "[deg]", 0));
        label_7->setText(QString());
        label_8->setText(QString());
        body_torque->setTitle(QApplication::translate("MainWindowDesign", "Torque On/Off", 0));
        empty_1->setText(QString());
        empty_3->setText(QString());
        empty_2->setText(QString());
        tab->setTabText(tab->indexOf(body_tab), QApplication::translate("MainWindowDesign", "Body", 0));
        inipose_button->setText(QApplication::translate("MainWindowDesign", "Initial Pose", 0));
        refresh_button->setText(QApplication::translate("MainWindowDesign", "Refresh", 0));
        save_offset_button->setText(QApplication::translate("MainWindowDesign", "Save Offset", 0));
        save_gain_button->setText(QApplication::translate("MainWindowDesign", "Save Gain", 0));
        tuning_pose_button->setText(QApplication::translate("MainWindowDesign", "Tuning Pose", 0));
        tuning_pose_comboBox->clear();
        tuning_pose_comboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindowDesign", "tune_pose_01", 0)
         << QApplication::translate("MainWindowDesign", "tune_pose_02", 0)
         << QApplication::translate("MainWindowDesign", "tune_pose_03", 0)
         << QApplication::translate("MainWindowDesign", "tune_pose_04", 0)
        );
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
