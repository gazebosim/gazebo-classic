#ifndef _BUILDING_ITEM_HH_
#define _BUILDING_ITEM_HH_

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class BuildingItem
    {
        public: BuildingItem();

        public: ~BuildingItem();

        public: int GetLevel();

        public: void SetLevel(int _level);

        public: double GetLevelBaseHeight();

        public: void SetLevelBaseHeight(double _height);

        protected: int level;

        protected: double levelBaseHeight;
    };
  }
}

#endif
