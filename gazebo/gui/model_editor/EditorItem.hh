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

        signals: void sizeChanged(double _width, double _length,
            double _height);

        signals: void poseChanged(double _x, double _y, double _z,
            double _roll, double _pitch, double _yaw);
    };
  }
}

#endif
