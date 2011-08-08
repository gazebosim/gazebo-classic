#ifndef INSERT_MODEL_WIDGET_HH
#define INSERT_MODEL_WIDGET_HH

#include <QWidget>

#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;

namespace gazebo
{
  namespace gui
  {
    class InsertModelWidget : public QWidget
    {
      Q_OBJECT
      public: InsertModelWidget( QWidget *parent = 0 );
      public: virtual ~InsertModelWidget();

      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);
      private slots: void OnApply();
      private slots: void OnCancel();

      private: QTreeWidget *fileTreeWidget;
      private: QPushButton *addButton;
      private: QPushButton *cancelButton;

      private: transport::NodePtr node;
      private: transport::PublisherPtr factoryPub, visualPub, selectionPub;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;
      private: std::string selectedModel;
    };
  }
}
#endif
