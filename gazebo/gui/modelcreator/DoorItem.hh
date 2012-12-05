
#ifndef _DOOR_ITEM_H_
#define _DOOR_ITEM_H_

#include "gui/qt.h"

class RectItem;

class DoorItem : public RectItem
{
    public: DoorItem();

    public: ~DoorItem();

    private: virtual void paint (QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);
};

#endif
