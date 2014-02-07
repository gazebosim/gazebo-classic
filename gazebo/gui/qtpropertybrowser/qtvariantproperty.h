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

#ifndef QTVARIANTPROPERTY_H
#define QTVARIANTPROPERTY_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>

#include "qteditorfactory.h"
#include "qtpropertybrowser.h"

#if QT_VERSION >= 0x040400
QT_BEGIN_NAMESPACE
#endif

typedef QMap<int, QIcon> QtIconMap;

class QtVariantPropertyManager;
class QtVariantPropertyPrivate;

class QT_QTPROPERTYBROWSER_EXPORT QtVariantProperty : public QtProperty
{
  public:
    ~QtVariantProperty();
    QVariant value() const;
    QVariant attributeValue(const QString &attribute) const;
    int valueType() const;
    int propertyType() const;

    void setValue(const QVariant &value);
    void setAttribute(const QString &attribute, const QVariant &value);
  protected:
    explicit QtVariantProperty(QtVariantPropertyManager *manager);
  private:
    friend class QtVariantPropertyManager;
    QtVariantPropertyPrivate *d_ptr;
};

class QtVariantPropertyPrivate
{
  QtVariantProperty *q_ptr;
  public:
  explicit QtVariantPropertyPrivate(QtVariantPropertyManager *m)
    : q_ptr(NULL), manager(m) {}

  QtVariantPropertyManager *manager;
};



class QtVariantPropertyManagerPrivate;

class QT_QTPROPERTYBROWSER_EXPORT QtVariantPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtVariantPropertyManager(QObject *parent = 0);
    ~QtVariantPropertyManager();

    virtual QtVariantProperty *addProperty(int propertyType,
        const QString &name = QString());

    int propertyType(const QtProperty *property) const;
    int valueType(const QtProperty *property) const;
    QtVariantProperty *variantProperty(const QtProperty *property) const;

    virtual bool isPropertyTypeSupported(int propertyType) const;
    virtual int valueType(int propertyType) const;
    virtual QStringList attributes(int propertyType) const;
    virtual int attributeType(int propertyType, const QString &attribute) const;

    virtual QVariant value(const QtProperty *property) const;
    virtual QVariant attributeValue(const QtProperty *property,
        const QString &attribute) const;

    static int enumTypeId();
    static int flagTypeId();
    static int groupTypeId();
    static int iconMapTypeId();
    public Q_SLOTS:
      virtual void setValue(QtProperty *property, const QVariant &val);
    virtual void setAttribute(QtProperty *property,
        const QString &attribute, const QVariant &value);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QVariant &val);
    void attributeChanged(QtProperty *property,
        const QString &attribute, const QVariant &val);
  protected:
    virtual bool hasValue(const QtProperty *property) const;
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
    virtual QtProperty *createProperty();
  private:
    QtVariantPropertyManagerPrivate *d_ptr;
    Q_PRIVATE_SLOT(d_func(), void slotValueChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotRangeChanged(QtProperty *, int, int))
      Q_PRIVATE_SLOT(d_func(), void slotSingleStepChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotValueChanged(QtProperty *, double))
      Q_PRIVATE_SLOT(d_func(),
          void slotRangeChanged(QtProperty *, double, double))
      Q_PRIVATE_SLOT(d_func(),
          void slotSingleStepChanged(QtProperty *, double))
      Q_PRIVATE_SLOT(d_func(),
          void slotDecimalsChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, bool))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QString &))
      Q_PRIVATE_SLOT(d_func(),
          void slotRegExpChanged(QtProperty *, const QRegExp &))
      Q_PRIVATE_SLOT(d_func(),
          void slotEchoModeChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QDate &))
      Q_PRIVATE_SLOT(d_func(),
          void slotRangeChanged(QtProperty *, const QDate &, const QDate &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QTime &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QDateTime &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QKeySequence &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QChar &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QLocale &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QPoint &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QPointF &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QSize &))
      Q_PRIVATE_SLOT(d_func(),
          void slotRangeChanged(QtProperty *, const QSize &, const QSize &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QSizeF &))
      Q_PRIVATE_SLOT(d_func(),
          void slotRangeChanged(QtProperty *, const QSizeF &, const QSizeF &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QRect &))
      Q_PRIVATE_SLOT(d_func(),
          void slotConstraintChanged(QtProperty *, const QRect &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QRectF &))
      Q_PRIVATE_SLOT(d_func(),
          void slotConstraintChanged(QtProperty *, const QRectF &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QColor &))
      Q_PRIVATE_SLOT(d_func(),
          void slotEnumNamesChanged(QtProperty *, const QStringList &))
      Q_PRIVATE_SLOT(d_func(),
          void slotEnumIconsChanged(QtProperty *, const QMap<int, QIcon> &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QSizePolicy &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QFont &))
      Q_PRIVATE_SLOT(d_func(),
          void slotValueChanged(QtProperty *, const QCursor &))
      Q_PRIVATE_SLOT(d_func(),
          void slotFlagNamesChanged(QtProperty *, const QStringList &))

      Q_PRIVATE_SLOT(d_func(),
          void slotPropertyInserted(QtProperty *, QtProperty *, QtProperty *))
      Q_PRIVATE_SLOT(d_func(),
          void slotPropertyRemoved(QtProperty *, QtProperty *))
      Q_DECLARE_PRIVATE(QtVariantPropertyManager)
      Q_DISABLE_COPY(QtVariantPropertyManager)
};

class QtVariantPropertyManagerPrivate
{
  QtVariantPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtVariantPropertyManager)
  public:
    QtVariantPropertyManagerPrivate();

    bool m_creatingProperty;
    bool m_creatingSubProperties;
    bool m_destroyingSubProperties;
    int m_propertyType;

    void slotValueChanged(QtProperty *property, int val);
    void slotRangeChanged(QtProperty *property, int min, int max);
    void slotSingleStepChanged(QtProperty *property, int step);
    void slotValueChanged(QtProperty *property, double val);
    void slotRangeChanged(QtProperty *property, double min, double max);
    void slotSingleStepChanged(QtProperty *property, double step);
    void slotDecimalsChanged(QtProperty *property, int prec);
    void slotValueChanged(QtProperty *property, bool val);
    void slotValueChanged(QtProperty *property, const QString &val);
    void slotRegExpChanged(QtProperty *property, const QRegExp &regExp);
    void slotEchoModeChanged(QtProperty *property, int);
    void slotValueChanged(QtProperty *property, const QDate &val);
    void slotRangeChanged(QtProperty *property,
        const QDate &min, const QDate &max);
    void slotValueChanged(QtProperty *property, const QTime &val);
    void slotValueChanged(QtProperty *property, const QDateTime &val);
    void slotValueChanged(QtProperty *property, const QKeySequence &val);
    void slotValueChanged(QtProperty *property, const QChar &val);
    void slotValueChanged(QtProperty *property, const QLocale &val);
    void slotValueChanged(QtProperty *property, const QPoint &val);
    void slotValueChanged(QtProperty *property, const QPointF &val);
    void slotValueChanged(QtProperty *property, const QSize &val);
    void slotRangeChanged(QtProperty *property,
        const QSize &min, const QSize &max);
    void slotValueChanged(QtProperty *property, const QSizeF &val);
    void slotRangeChanged(QtProperty *property, const QSizeF &min,
        const QSizeF &max);
    void slotValueChanged(QtProperty *property, const QRect &val);
    void slotConstraintChanged(QtProperty *property, const QRect &val);
    void slotValueChanged(QtProperty *property, const QRectF &val);
    void slotConstraintChanged(QtProperty *property, const QRectF &val);
    void slotValueChanged(QtProperty *property, const QColor &val);
    void slotEnumChanged(QtProperty *property, int val);
    void slotEnumNamesChanged(QtProperty *property,
        const QStringList &enumNames);
    void slotEnumIconsChanged(QtProperty *property,
        const QMap<int, QIcon> &enumIcons);
    void slotValueChanged(QtProperty *property, const QSizePolicy &val);
    void slotValueChanged(QtProperty *property, const QFont &val);
    void slotValueChanged(QtProperty *property, const QCursor &val);
    void slotFlagChanged(QtProperty *property, int val);
    void slotFlagNamesChanged(QtProperty *property,
        const QStringList &flagNames);
    void slotPropertyInserted(QtProperty *property, QtProperty *parent,
        QtProperty *after);
    void slotPropertyRemoved(QtProperty *property, QtProperty *parent);

    void valueChanged(QtProperty *property, const QVariant &val);

    int internalPropertyToType(QtProperty *property) const;
    QtVariantProperty *createSubProperty(QtVariantProperty *parent,
        QtVariantProperty *after, QtProperty *internal);
    void removeSubProperty(QtVariantProperty *property);

    QMap<int, QtAbstractPropertyManager *> m_typeToPropertyManager;
    QMap<int, QMap<QString, int> > m_typeToAttributeToAttributeType;

    QMap<const QtProperty *, QPair<QtVariantProperty *, int> >
      m_propertyToType;

    QMap<int, int> m_typeToValueType;


    QMap<QtProperty *, QtVariantProperty *> m_internalToProperty;

    const QString m_constraintAttribute;
    const QString m_singleStepAttribute;
    const QString m_decimalsAttribute;
    const QString m_enumIconsAttribute;
    const QString m_enumNamesAttribute;
    const QString m_flagNamesAttribute;
    const QString m_maximumAttribute;
    const QString m_minimumAttribute;
    const QString m_regExpAttribute;
    const QString m_echoModeAttribute;
};
class QtVariantEditorFactoryPrivate;

class QT_QTPROPERTYBROWSER_EXPORT QtVariantEditorFactory
: public QtAbstractEditorFactory<QtVariantPropertyManager>
{
  Q_OBJECT
  public:
    explicit QtVariantEditorFactory(QObject *parent = 0);
    ~QtVariantEditorFactory();
  protected:
    void connectPropertyManager(QtVariantPropertyManager *manager);
    QWidget *createEditor(QtVariantPropertyManager *manager,
        QtProperty *property, QWidget *parent);
    void disconnectPropertyManager(QtVariantPropertyManager *manager);
  private:
    QtVariantEditorFactoryPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtVariantEditorFactory)
      Q_DISABLE_COPY(QtVariantEditorFactory)
};

class QtVariantEditorFactoryPrivate
{
  QtVariantEditorFactory *q_ptr;
  Q_DECLARE_PUBLIC(QtVariantEditorFactory)
  public:
    QtVariantEditorFactoryPrivate()
      : q_ptr(NULL),
      m_spinBoxFactory(NULL),
      m_doubleSpinBoxFactory(NULL),
      m_checkBoxFactory(NULL),
      m_lineEditFactory(NULL),
      m_dateEditFactory(NULL),
      m_timeEditFactory(NULL),
      m_dateTimeEditFactory(NULL),
      m_keySequenceEditorFactory(NULL),
      m_charEditorFactory(NULL),
      m_comboBoxFactory(NULL),
      m_cursorEditorFactory(NULL),
      m_colorEditorFactory(NULL),
      m_fontEditorFactory(NULL) {}


    QtSpinBoxFactory           *m_spinBoxFactory;
    QtDoubleSpinBoxFactory     *m_doubleSpinBoxFactory;
    QtCheckBoxFactory          *m_checkBoxFactory;
    QtLineEditFactory          *m_lineEditFactory;
    QtDateEditFactory          *m_dateEditFactory;
    QtTimeEditFactory          *m_timeEditFactory;
    QtDateTimeEditFactory      *m_dateTimeEditFactory;
    QtKeySequenceEditorFactory *m_keySequenceEditorFactory;
    QtCharEditorFactory        *m_charEditorFactory;
    QtEnumEditorFactory        *m_comboBoxFactory;
    QtCursorEditorFactory      *m_cursorEditorFactory;
    QtColorEditorFactory       *m_colorEditorFactory;
    QtFontEditorFactory        *m_fontEditorFactory;

    QMap<QtAbstractEditorFactoryBase *, int> m_factoryToType;
    QMap<int, QtAbstractEditorFactoryBase *> m_typeToFactory;
};

class QtEnumPropertyType
{
};


class QtFlagPropertyType
{
};


class QtGroupPropertyType
{
};

  Q_DECLARE_METATYPE(QtEnumPropertyType)
  Q_DECLARE_METATYPE(QtFlagPropertyType)
Q_DECLARE_METATYPE(QtGroupPropertyType)



#if QT_VERSION >= 0x040400
  QT_END_NAMESPACE
#endif

  Q_DECLARE_METATYPE(QIcon)
Q_DECLARE_METATYPE(QtIconMap)
#endif
