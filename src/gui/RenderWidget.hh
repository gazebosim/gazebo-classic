#ifndef RENDER_WIDGET_HH
#define RENDER_WIDGET_HH

#include <QWidget>
#include "common/Event.hh"

class QLineEdit;
class QLabel;
class QFrame;
class QHBoxLayout;

namespace gazebo
{
  class GLWidget;

  namespace gui
  {
    class RenderWidget : public QWidget
    {
      Q_OBJECT
      public: RenderWidget( QWidget *parent = 0 );
      public: virtual ~RenderWidget();

      public: void RemoveScene(const std::string &_name);
      public: void CreateScene(const std::string &_name);

      private slots: virtual void update();

      private: void OnFullScreen(bool &_value);

      private: QHBoxLayout *bottomBarLayout;
      private: GLWidget *glWidget;
      private: QFrame *mainFrame;
      private: QLabel *xyzLabel;
      private: QLineEdit *xPosEdit;
      private: QLineEdit *yPosEdit;
      private: QLineEdit *zPosEdit;

      private: QLabel *rpyLabel;
      private: QLineEdit *rollEdit;
      private: QLineEdit *pitchEdit;
      private: QLineEdit *yawEdit;
      private: QLineEdit *fpsEdit;
      private: QLineEdit *trianglesEdit;

      private: std::vector<event::ConnectionPtr> connections;

      private: bool clear;
      private: std::string clearName;

      private: bool create;
      private: std::string createName;
      private: QTimer *timer;
    };
  }
}
#endif
