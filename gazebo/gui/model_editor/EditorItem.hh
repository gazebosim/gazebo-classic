#ifndef _EDITOR_ITEM_HH_
#define _EDITOR_ITEM_HH_

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class EditorItem : public QObject
    {
        Q_OBJECT

        public: EditorItem();

        public: ~EditorItem();

        public: virtual QVector3D GetSize();

        public: virtual QVector3D GetScenePosition();

        public: virtual double GetSceneRotation();

        signals: void sizeChanged(double _width, double _depth,
            double _height);

        signals: void poseChanged(double _x, double _y, double _z,
            double _roll, double _pitch, double _yaw);

        signals: void poseOriginTransformed(double _x, double _y, double _z,
            double _roll, double _pitch, double _yaw);

        signals: void positionChanged(double _x, double _y, double _z);

        signals: void widthChanged(double _width);

        signals: void depthChanged(double _depth);

        signals: void heightChanged(double _height);

        signals: void posXChanged(double _posX);

        signals: void posYChanged(double _posY);

        signals: void posZChanged(double _posX);

        signals: void yawChanged(double _yaw);

        signals: void originChanged(double _xRatio, double _yRatio,
            double _zRatio);

        private: double baseHeight;
    };
  }
}

#endif
