#ifndef EDIT_MODEL_WIDGET_HH
#define EDIT_MODEL_WIDGET_HH

#include <QWidget>
#include <QPushButton>
#include <QFrame>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QGroupBox>

#include "transport/TransportTypes.hh"
#include "msgs/msgs.h"

class QTreeWidget;

namespace gazebo
{
  namespace gui
  {
    class ModelEditWidget : public QWidget
    {
      Q_OBJECT
      public: ModelEditWidget( QWidget *parent = 0 );
      public: virtual ~ModelEditWidget();

      protected: void closeEvent(QCloseEvent * /*_event*/);
      protected: void showEvent(QShowEvent * /*_event*/);
      private: QTreeWidget *treeWidget;
    };

    class ModelPropertyWidget : public QWidget
    {
      Q_OBJECT
      public: ModelPropertyWidget( QWidget *parent = 0 );
      public: virtual ~ModelPropertyWidget();

      private: QLineEdit *nameEdit;
      private: QLineEdit *xEdit, *yEdit, *zEdit;
      private: QLineEdit *rollEdit, *pitchEdit, *yawEdit;
      private: QGroupBox *originBox;
      private: QCheckBox *staticCheck;
    };

  }
}

#endif
