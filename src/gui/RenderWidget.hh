#ifndef RENDER_WIDGET_HH
#define RENDER_WIDGET_HH

#include <QWidget>
class QLineEdit;

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

      private: GLWidget *glWidget;

      private: QLineEdit *xPosEdit;
      private: QLineEdit *yPosEdit;
      private: QLineEdit *zPosEdit;

      private: QLineEdit *rollEdit;
      private: QLineEdit *pitchEdit;
      private: QLineEdit *yawEdit;
      private: QLineEdit *fpsOut;
      private: QLineEdit *trianglesOut;

    };
  }
}
#endif
