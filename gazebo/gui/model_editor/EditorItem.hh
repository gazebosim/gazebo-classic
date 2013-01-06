#ifndef _EDITOR_ITEM_HH_
#define _EDITOR_ITEM_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class EditorItem : public QObject
    {
        Q_OBJECT

        public: EditorItem();

        public: ~EditorItem();

        public: virtual QVector3D GetSize() const;

        public: virtual QVector3D GetScenePosition() const;

        public: virtual double GetSceneRotation() const;

        public: virtual std::string GetType() const;

        Q_SIGNALS: void sizeChanged(double _width, double _depth,
            double _height);

        Q_SIGNALS: void poseChanged(double _x, double _y, double _z,
            double _roll, double _pitch, double _yaw);

        Q_SIGNALS: void poseOriginTransformed(double _x, double _y, double _z,
            double _roll, double _pitch, double _yaw);

        Q_SIGNALS: void positionChanged(double _x, double _y, double _z);

        Q_SIGNALS: void rotationChanged(double _roll, double _pitch, double _yaw);

        Q_SIGNALS: void widthChanged(double _width);

        Q_SIGNALS: void depthChanged(double _depth);

        Q_SIGNALS: void heightChanged(double _height);

        Q_SIGNALS: void posXChanged(double _posX);

        Q_SIGNALS: void posYChanged(double _posY);

        Q_SIGNALS: void posZChanged(double _posX);

        Q_SIGNALS: void yawChanged(double _yaw);

        Q_SIGNALS: void originChanged(double _xRatio, double _yRatio,
            double _zRatio);

        Q_SIGNALS: void itemDeleted();

        protected: std::string editorType;
    };
  }
}

#endif
