
#ifndef _WINDOW_ITEM_H_
#define _WINDOW_ITEM_H_

#include "gui/qt.h"

class RectItem;

class WindowItem : public RectItem
{
    public: WindowItem();

    public: ~WindowItem();

    private: virtual void paint (QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);
};

#endif
