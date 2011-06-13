#ifndef INSERT_MODEL_WIDGET_HH
#define INSERT_MODEL_WIDGET_HH

#include <QWidget>

class QTreeWidget;

namespace gazebo
{
  namespace gui
  {
    class InsertModelWidget : public QWidget
    {
      Q_OBJECT
      public: InsertModelWidget( QWidget *parent = 0 );
      public: virtual ~InsertModelWidget();

      private: QTreeWidget *fileTreeWidget;
    };
  }
}
#endif
