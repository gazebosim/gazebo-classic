#ifndef GL_WIDGET_HH
#define GL_WIDGET_HH

#include <QtGui>

#include "rendering/RenderTypes.hh"
#include "common/MouseEvent.hh"

#include "gui/BoxMaker.hh"

namespace gazebo
{
  namespace rendering
  {
    class UserCamera;
  }

  namespace gui
  {
    class GLWidget : public QWidget
    {
      Q_OBJECT

      public: GLWidget(QWidget *parent=0);
      public: virtual ~GLWidget();

      public: void ViewScene(rendering::ScenePtr scene);
      public: rendering::UserCameraPtr GetCamera() const;

      public: void CreateEntity(const std::string &name);

      signals:
        void clicked();

      protected: virtual void moveEvent(QMoveEvent *e);
      protected: virtual void paintEvent(QPaintEvent *e);
      protected: virtual void resizeEvent(QResizeEvent *e);
      protected: virtual void showEvent(QShowEvent *e);


      protected: void wheelEvent(QWheelEvent *event);
      protected: void mousePressEvent(QMouseEvent *event);
      protected: void mouseMoveEvent(QMouseEvent *event);
      protected: void mouseReleaseEvent(QMouseEvent *event);

      private: std::string GetOgreHandle() const;

      private: int windowId;

      private: rendering::UserCameraPtr userCamera;
      private: QFrame *renderFrame;
      private: common::MouseEvent mouseEvent;


      private: EntityMaker *entityMaker;
      private: BoxMaker boxMaker;
    };
  }
}

#endif
