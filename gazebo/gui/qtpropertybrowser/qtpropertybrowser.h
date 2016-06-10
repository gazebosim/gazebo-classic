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

#ifndef QTPROPERTYBROWSER_H
#define QTPROPERTYBROWSER_H

#include <QtGui/QWidget>
#include <QtCore/QSet>
#include <QtGui/QLineEdit>
#include <QtCore/QMap>

#if QT_VERSION >= 0x040400
QT_BEGIN_NAMESPACE
#endif

#if defined(Q_WS_WIN)
#  if !defined(QT_QTPROPERTYBROWSER_EXPORT) && \
!defined(QT_QTPROPERTYBROWSER_IMPORT)
#    define QT_QTPROPERTYBROWSER_EXPORT
#  elif defined(QT_QTPROPERTYBROWSER_IMPORT)
#    if defined(QT_QTPROPERTYBROWSER_EXPORT)
#      undef QT_QTPROPERTYBROWSER_EXPORT
#    endif
#    define QT_QTPROPERTYBROWSER_EXPORT __declspec(dllimport)
#  elif defined(QT_QTPROPERTYBROWSER_EXPORT)
#    undef QT_QTPROPERTYBROWSER_EXPORT
#    define QT_QTPROPERTYBROWSER_EXPORT __declspec(dllexport)
#  endif
#else
#  define QT_QTPROPERTYBROWSER_EXPORT
#endif

typedef QLineEdit::EchoMode EchoMode;

class QtAbstractPropertyManager;
class QtPropertyPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtProperty
{
  public:
    virtual ~QtProperty();

    QList<QtProperty *> subProperties() const;

    QtAbstractPropertyManager *propertyManager() const;

    QString toolTip() const;
    QString statusTip() const;
    QString whatsThis() const;
    QString propertyName() const;
    bool isEnabled() const;
    bool isModified() const;

    bool hasValue() const;
    QIcon valueIcon() const;
    QString valueText() const;
    QString displayText() const;

    void setToolTip(const QString &text);
    void setStatusTip(const QString &text);
    void setWhatsThis(const QString &text);
    void setPropertyName(const QString &text);
    void setEnabled(bool enable);
    void setModified(bool modified);

    void addSubProperty(QtProperty *property);
    void insertSubProperty(QtProperty *property, QtProperty *afterProperty);
    void removeSubProperty(QtProperty *property);
  protected:
    explicit QtProperty(QtAbstractPropertyManager *manager);
    void propertyChanged();
  private:
    friend class QtAbstractPropertyManager;
    QtPropertyPrivate *d_ptr;
};

class QtPropertyPrivate
{
  public:
    explicit QtPropertyPrivate(QtAbstractPropertyManager *manager)
      : q_ptr(NULL),
      m_enabled(true),
      m_modified(false),
      m_manager(manager) {}
    QtProperty *q_ptr;

    QSet<QtProperty *> m_parentItems;
    QList<QtProperty *> m_subItems;

    QString m_toolTip;
    QString m_statusTip;
    QString m_whatsThis;
    QString m_name;
    bool m_enabled;
    bool m_modified;

    QtAbstractPropertyManager * const m_manager;
};

class QtAbstractPropertyManagerPrivate;

class QT_QTPROPERTYBROWSER_EXPORT QtAbstractPropertyManager : public QObject
{
  Q_OBJECT
  public:

    explicit QtAbstractPropertyManager(QObject *parent = 0);
    ~QtAbstractPropertyManager();

    QSet<QtProperty *> properties() const;
    void clear() const;

    QtProperty *addProperty(const QString &name = QString());
  Q_SIGNALS:

    void propertyInserted(QtProperty *property,
        QtProperty *parent, QtProperty *after);
    void propertyChanged(QtProperty *property);
    void propertyRemoved(QtProperty *property, QtProperty *parent);
    void propertyDestroyed(QtProperty *property);
  protected:
    virtual bool hasValue(const QtProperty *property) const;
    virtual QIcon valueIcon(const QtProperty *property) const;
    virtual QString valueText(const QtProperty *property) const;
    virtual QString displayText(const QtProperty *property) const;
    virtual EchoMode echoMode(const QtProperty *) const;
    virtual void initializeProperty(QtProperty *property) = 0;
    virtual void uninitializeProperty(QtProperty *property);
    virtual QtProperty *createProperty();
  private:
    friend class QtProperty;
    QtAbstractPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtAbstractPropertyManager)
      Q_DISABLE_COPY(QtAbstractPropertyManager)
};

class QtAbstractPropertyManagerPrivate
{
  QtAbstractPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtAbstractPropertyManager)
  public:
    QtAbstractPropertyManagerPrivate() : q_ptr(NULL) {}
    void propertyDestroyed(QtProperty *property);
    void propertyChanged(QtProperty *property) const;
    void propertyRemoved(QtProperty *property,
        QtProperty *parentProperty) const;
    void propertyInserted(QtProperty *property, QtProperty *parentProperty,
        QtProperty *afterProperty) const;

    QSet<QtProperty *> m_properties;
};

class QT_QTPROPERTYBROWSER_EXPORT QtAbstractEditorFactoryBase : public QObject
{
  Q_OBJECT
  public:
    virtual QWidget *createEditor(QtProperty *property, QWidget *parent) = 0;
  protected:
    explicit QtAbstractEditorFactoryBase(QObject *parent = 0)
      : QObject(parent) {}

    virtual void breakConnection(QtAbstractPropertyManager *manager) = 0;
    protected Q_SLOTS:
      virtual void managerDestroyed(QObject *manager) = 0;

    friend class QtAbstractPropertyBrowser;
};

template <class PropertyManager>
class QtAbstractEditorFactory : public QtAbstractEditorFactoryBase
{
  public:
    explicit QtAbstractEditorFactory(
        QObject *parent) : QtAbstractEditorFactoryBase(parent) {}
    QWidget *createEditor(QtProperty *property, QWidget *parent)

    {
      QSetIterator<PropertyManager *> it(m_managers);
      while (it.hasNext())
      {
        PropertyManager *manager = it.next();
        if (manager == property->propertyManager())
        {
          return createEditor(manager, property, parent);
        }
      }
      return 0;
    }
    void addPropertyManager(PropertyManager *manager)

    {
      if (m_managers.contains(manager))
        return;
      m_managers.insert(manager);
      connectPropertyManager(manager);
      connect(manager, SIGNAL(destroyed(QObject *)),
          this, SLOT(managerDestroyed(QObject *)));
    }
    void removePropertyManager(PropertyManager *manager)

    {
      if (!m_managers.contains(manager))
        return;
      disconnect(manager, SIGNAL(destroyed(QObject *)),
          this, SLOT(managerDestroyed(QObject *)));
      disconnectPropertyManager(manager);
      m_managers.remove(manager);
    }
    QSet<PropertyManager *> propertyManagers() const

    {
      return m_managers;
    }
    PropertyManager *propertyManager(QtProperty *property) const

    {
      QtAbstractPropertyManager *manager = property->propertyManager();
      QSetIterator<PropertyManager *> itManager(m_managers);
      while (itManager.hasNext())
      {
        PropertyManager *m = itManager.next();
        if (m == manager)
        {
          return m;
        }
      }
      return 0;
    }
  protected:
    virtual void connectPropertyManager(PropertyManager *manager) = 0;
    virtual QWidget *createEditor(PropertyManager *manager,
        QtProperty *property,
        QWidget *parent) = 0;
    virtual void disconnectPropertyManager(PropertyManager *manager) = 0;
    void managerDestroyed(QObject *manager)

    {
      QSetIterator<PropertyManager *> it(m_managers);
      while (it.hasNext())
      {
        PropertyManager *m = it.next();
        if (m == manager)
        {
          m_managers.remove(m);
          return;
        }
      }
    }
  private:
    void breakConnection(QtAbstractPropertyManager *manager)

    {
      QSetIterator<PropertyManager *> it(m_managers);
      while (it.hasNext())
      {
        PropertyManager *m = it.next();
        if (m == manager)
        {
          removePropertyManager(m);
          return;
        }
      }
    }
  private:
    QSet<PropertyManager *> m_managers;
    friend class QtAbstractPropertyEditor;
};

class QtAbstractPropertyBrowser;
class QtBrowserItemPrivate;

class QT_QTPROPERTYBROWSER_EXPORT QtBrowserItem
{
  public:
    QtProperty *property() const;
    QtBrowserItem *parent() const;
    QList<QtBrowserItem *> children() const;
    QtAbstractPropertyBrowser *browser() const;
  private:
    explicit QtBrowserItem(QtAbstractPropertyBrowser *browser,
        QtProperty *property, QtBrowserItem *parent);
    ~QtBrowserItem();
    QtBrowserItemPrivate *d_ptr;
    friend class QtAbstractPropertyBrowserPrivate;
};

class QtBrowserItemPrivate
{
  public:
    QtBrowserItemPrivate(QtAbstractPropertyBrowser *browser,
        QtProperty *property, QtBrowserItem *parent)
      : m_browser(browser), m_property(property), m_parent(parent), q_ptr(0) {}

    void addChild(QtBrowserItem *index, QtBrowserItem *after);
    void removeChild(QtBrowserItem *index);

    QtAbstractPropertyBrowser * const m_browser;
    QtProperty *m_property;
    QtBrowserItem *m_parent;

    QtBrowserItem *q_ptr;

    QList<QtBrowserItem *> m_children;
};

class QtAbstractPropertyBrowserPrivate
{
  QtAbstractPropertyBrowser *q_ptr;
  Q_DECLARE_PUBLIC(QtAbstractPropertyBrowser)
  public:
    QtAbstractPropertyBrowserPrivate();

    void insertSubTree(QtProperty *property,
        QtProperty *parentProperty);
    void removeSubTree(QtProperty *property,
        QtProperty *parentProperty);
    void createBrowserIndexes(QtProperty *property, QtProperty *parentProperty,
        QtProperty *afterProperty);
    void removeBrowserIndexes(QtProperty *property, QtProperty *parentProperty);
    QtBrowserItem *createBrowserIndex(QtProperty *property,
        QtBrowserItem *parentIndex, QtBrowserItem *afterIndex);
    void removeBrowserIndex(QtBrowserItem *index);
    void clearIndex(QtBrowserItem *index);

    void slotPropertyInserted(QtProperty *property,
        QtProperty *parentProperty, QtProperty *afterProperty);
    void slotPropertyRemoved(QtProperty *property, QtProperty *parentProperty);
    void slotPropertyDestroyed(QtProperty *property);
    void slotPropertyDataChanged(QtProperty *property);

    QList<QtProperty *> m_subItems;
    QMap<QtAbstractPropertyManager *,
      QList<QtProperty *> > m_managerToProperties;
    QMap<QtProperty *, QList<QtProperty *> > m_propertyToParents;

    QMap<QtProperty *, QtBrowserItem *> m_topLevelPropertyToIndex;
    QList<QtBrowserItem *> m_topLevelIndexes;
    QMap<QtProperty *, QList<QtBrowserItem *> > m_propertyToIndexes;

    QtBrowserItem *m_currentItem;
};



class QT_QTPROPERTYBROWSER_EXPORT QtAbstractPropertyBrowser : public QWidget
{
  Q_OBJECT
  public:

    explicit QtAbstractPropertyBrowser(QWidget *parent = 0);
    ~QtAbstractPropertyBrowser();

    QList<QtProperty *> properties() const;
    QList<QtBrowserItem *> items(QtProperty *property) const;
    QtBrowserItem *topLevelItem(QtProperty *property) const;
    QList<QtBrowserItem *> topLevelItems() const;
    void clear();

    template <class PropertyManager>
      void setFactoryForManager(PropertyManager *manager,
          QtAbstractEditorFactory<PropertyManager> *factory)

      {
        QtAbstractPropertyManager *abstractManager = manager;
        QtAbstractEditorFactoryBase *abstractFactory = factory;

        if (addFactory(abstractManager, abstractFactory))
          factory->addPropertyManager(manager);
      }

    void unsetFactoryForManager(QtAbstractPropertyManager *manager);

    QtBrowserItem *currentItem() const;
    void setCurrentItem(QtBrowserItem *e);

    Q_SIGNALS:
    void currentItemChanged(QtBrowserItem *e);

    public Q_SLOTS:

      QtBrowserItem *addProperty(QtProperty *property);
    QtBrowserItem *insertProperty(QtProperty *property,
        QtProperty *afterProperty);
    void removeProperty(QtProperty *property);

  protected:

    virtual void itemInserted(QtBrowserItem *item,
        QtBrowserItem *afterItem) = 0;
    virtual void itemRemoved(QtBrowserItem *item) = 0;
    // can be tooltip, statustip, whatsthis, name, icon, text.
    virtual void itemChanged(QtBrowserItem *item) = 0;

    virtual QWidget *createEditor(QtProperty *property, QWidget *parent);
  private:

    bool addFactory(QtAbstractPropertyManager *abstractManager,
        QtAbstractEditorFactoryBase *abstractFactory);

    QtAbstractPropertyBrowserPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtAbstractPropertyBrowser)
      Q_DISABLE_COPY(QtAbstractPropertyBrowser)
      Q_PRIVATE_SLOT(d_func(), void slotPropertyInserted(QtProperty *,
            QtProperty *, QtProperty *))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyRemoved(QtProperty *,
            QtProperty *))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDataChanged(QtProperty *))
};

#if QT_VERSION >= 0x040400
QT_END_NAMESPACE
#endif

#endif  // QTPROPERTYBROWSER_H
