/****************************************************************************
 **
 ** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
 ** All rights reserved.
 **
 ** Contact: Nokia Corporation (qt-info@nokia.com)
 **
 ** This file is part of a Qt Solutions component.
 **
 ** You may use this file under the terms of the BSD license as follows:
 **
 ** "Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **   * Redistributions of source code must retain the above copyright
 **     notice, this list of conditions and the following disclaimer.
 **   * Redistributions in binary form must reproduce the above copyright
 **     notice, this list of conditions and the following disclaimer in
 **     the documentation and/or other materials provided with the
 **     distribution.
 **   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
 **     the names of its contributors may be used to endorse or promote
 **     products derived from this software without specific prior written
 **     permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 **
 ****************************************************************************/

#pragma GCC system_header
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wshadow"

#ifndef QTTREEPROPERTYBROWSER_H
#define QTTREEPROPERTYBROWSER_H

#include <QtCore/QModelIndex>
#include <QtGui/QTreeWidget>
#include <QtGui/QItemDelegate>
#include "qtpropertybrowser.h"

#if QT_VERSION >= 0x040400
QT_BEGIN_NAMESPACE
#endif

class QTreeWidgetItem;
class QtTreePropertyBrowserPrivate;

class QtPropertyEditorDelegate : public QItemDelegate
{
  Q_OBJECT
  public:
    explicit QtPropertyEditorDelegate(QObject *parent = 0)
      : QItemDelegate(parent), m_editorPrivate(0), m_editedItem(0),
      m_editedWidget(0), m_disablePainting(false)
  {}

    void setEditorPrivate(QtTreePropertyBrowserPrivate *editorPrivate)
    { m_editorPrivate = editorPrivate; }

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor,
        const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

    void setModelData(QWidget *, QAbstractItemModel *,
        const QModelIndex &) const {}

    void setEditorData(QWidget *, const QModelIndex &) const {}

    bool eventFilter(QObject *object, QEvent *event);
    void closeEditor(QtProperty *property);

    QTreeWidgetItem *editedItem() const { return m_editedItem; }

  protected:

    void drawDecoration(QPainter *painter, const QStyleOptionViewItem &option,
        const QRect &rect, const QPixmap &pixmap) const;
    void drawDisplay(QPainter *painter, const QStyleOptionViewItem &option,
        const QRect &rect, const QString &text) const;

    private slots:
      void slotEditorDestroyed(QObject *object);

  private:
    int indentation(const QModelIndex &index) const;

    typedef QMap<QWidget *, QtProperty *> EditorToPropertyMap;
    mutable EditorToPropertyMap m_editorToProperty;

    typedef QMap<QtProperty *, QWidget *> PropertyToEditorMap;
    mutable PropertyToEditorMap m_propertyToEditor;
    QtTreePropertyBrowserPrivate *m_editorPrivate;
    mutable QTreeWidgetItem *m_editedItem;
    mutable QWidget *m_editedWidget;
    mutable bool m_disablePainting;
};

class QtPropertyEditorView : public QTreeWidget
{
  Q_OBJECT
  public:
    explicit QtPropertyEditorView(QWidget *parent = 0);

    void setEditorPrivate(QtTreePropertyBrowserPrivate *editorPrivate)
    { m_editorPrivate = editorPrivate; }

    QTreeWidgetItem *indexToItem(const QModelIndex &index) const
    { return itemFromIndex(index); }

  protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void drawRow(QPainter *painter, const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

  private:
    QtTreePropertyBrowserPrivate *m_editorPrivate;
};


class QT_QTPROPERTYBROWSER_EXPORT QtTreePropertyBrowser
: public QtAbstractPropertyBrowser
{
  Q_OBJECT
    Q_ENUMS(ResizeMode)
    Q_PROPERTY(int indentation READ indentation WRITE setIndentation)
    Q_PROPERTY(
        bool rootIsDecorated READ rootIsDecorated WRITE setRootIsDecorated)
    Q_PROPERTY(
        bool alternatingRowColors READ
        alternatingRowColors WRITE setAlternatingRowColors)
    Q_PROPERTY(bool headerVisible READ isHeaderVisible WRITE setHeaderVisible)
    Q_PROPERTY(ResizeMode resizeMode READ resizeMode WRITE setResizeMode)
    Q_PROPERTY(
        int splitterPosition READ splitterPosition WRITE setSplitterPosition)
    Q_PROPERTY(
        bool propertiesWithoutValueMarked READ
        propertiesWithoutValueMarked WRITE setPropertiesWithoutValueMarked)
  public:

    enum ResizeMode
    {
      Interactive,
      Stretch,
      Fixed,
      ResizeToContents
    };

    explicit QtTreePropertyBrowser(QWidget *parent = 0);
    ~QtTreePropertyBrowser();

    int indentation() const;
    void setIndentation(int i);

    bool rootIsDecorated() const;
    void setRootIsDecorated(bool show);

    bool alternatingRowColors() const;
    void setAlternatingRowColors(bool enable);

    bool isHeaderVisible() const;
    void setHeaderVisible(bool visible);

    ResizeMode resizeMode() const;
    void setResizeMode(ResizeMode mode);

    int splitterPosition() const;
    void setSplitterPosition(int position);

    void setExpanded(QtBrowserItem *item, bool expanded);
    bool isExpanded(QtBrowserItem *item) const;

    bool isItemVisible(QtBrowserItem *item) const;
    void setItemVisible(QtBrowserItem *item, bool visible);

    void setBackgroundColor(QtBrowserItem *item, const QColor &color);
    QColor backgroundColor(QtBrowserItem *item) const;
    QColor calculatedBackgroundColor(QtBrowserItem *item) const;

    void setPropertiesWithoutValueMarked(bool mark);
    bool propertiesWithoutValueMarked() const;

    void editItem(QtBrowserItem *item);

  Q_SIGNALS:

    void collapsed(QtBrowserItem *item);
    void expanded(QtBrowserItem *item);

  protected:
    virtual void itemInserted(QtBrowserItem *item, QtBrowserItem *afterItem);
    virtual void itemRemoved(QtBrowserItem *item);
    virtual void itemChanged(QtBrowserItem *item);

  private:

    QtTreePropertyBrowserPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtTreePropertyBrowser)
      Q_DISABLE_COPY(QtTreePropertyBrowser)

      Q_PRIVATE_SLOT(d_func(), void slotCollapsed(const QModelIndex &t))
      Q_PRIVATE_SLOT(d_func(), void slotExpanded(const QModelIndex &t))
      Q_PRIVATE_SLOT(d_func(),
          void slotCurrentBrowserItemChanged(QtBrowserItem *t))
      Q_PRIVATE_SLOT(d_func(),
          void slotCurrentTreeItemChanged(
            QTreeWidgetItem *t, QTreeWidgetItem *t2))
};

class QtTreePropertyBrowserPrivate
{
  QtTreePropertyBrowser *q_ptr;
  Q_DECLARE_PUBLIC(QtTreePropertyBrowser)

  public:
    QtTreePropertyBrowserPrivate();
    void init(QWidget *parent);

    void propertyInserted(QtBrowserItem *index, QtBrowserItem *afterIndex);
    void propertyRemoved(QtBrowserItem *index);
    void propertyChanged(QtBrowserItem *index);
    QWidget *createEditor(QtProperty *property, QWidget *parent) const
    { return q_ptr->createEditor(property, parent); }
    QtProperty *indexToProperty(const QModelIndex &index) const;
    QTreeWidgetItem *indexToItem(const QModelIndex &index) const;
    QtBrowserItem *indexToBrowserItem(const QModelIndex &index) const;
    bool lastColumn(int column) const;
    void disableItem(QTreeWidgetItem *item) const;
    void enableItem(QTreeWidgetItem *item) const;
    bool hasValue(QTreeWidgetItem *item) const;

    void slotCollapsed(const QModelIndex &index);
    void slotExpanded(const QModelIndex &index);

    QColor calculatedBackgroundColor(QtBrowserItem *item) const;

    QtPropertyEditorView *treeWidget() const { return m_treeWidget; }
    bool markPropertiesWithoutValue(
        ) const { return m_markPropertiesWithoutValue; }

    QtBrowserItem *currentItem() const;
    void setCurrentItem(QtBrowserItem *browserItem, bool block);
    void editItem(QtBrowserItem *browserItem);

    void slotCurrentBrowserItemChanged(QtBrowserItem *item);
    void slotCurrentTreeItemChanged(QTreeWidgetItem *newItem,
        QTreeWidgetItem *);

    QTreeWidgetItem *editedItem() const;

  private:
    void updateItem(QTreeWidgetItem *item);

    QMap<QtBrowserItem *, QTreeWidgetItem *> m_indexToItem;
    QMap<QTreeWidgetItem *, QtBrowserItem *> m_itemToIndex;

    QMap<QtBrowserItem *, QColor> m_indexToBackgroundColor;

    QtPropertyEditorView *m_treeWidget;

    bool m_headerVisible;
    QtTreePropertyBrowser::ResizeMode m_resizeMode;
    class QtPropertyEditorDelegate *m_delegate;
    bool m_markPropertiesWithoutValue;
    bool m_browserChangedBlocked;
    QIcon m_expandIcon;
};
#if QT_VERSION >= 0x040400
QT_END_NAMESPACE
#endif

#endif
