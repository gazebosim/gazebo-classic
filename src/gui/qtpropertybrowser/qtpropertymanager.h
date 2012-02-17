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


#ifndef QTPROPERTYMANAGER_H
#define QTPROPERTYMANAGER_H

#include <QtCore/QLocale>
#include <QtCore/QDateTime>
#include <QtCore/QMetaEnum>
#include <QLineEdit>
#include <QtGui/QIcon>
#include <climits>

#include "qtpropertybrowser.h"

#if QT_VERSION >= 0x040400
QT_BEGIN_NAMESPACE
#endif

class QDate;
class QTime;
class QDateTime;
class QLocale;

  template <class PrivateData, class Value>
static void setSimpleMinimumData(PrivateData *data, const Value &minVal)
{
  data->minVal = minVal;
  if (data->maxVal < data->minVal)
    data->maxVal = data->minVal;

  if (data->val < data->minVal)
    data->val = data->minVal;
}

  template <class PrivateData, class Value>
static void setSimpleMaximumData(PrivateData *data, const Value &maxVal)
{
  data->maxVal = maxVal;
  if (data->minVal > data->maxVal)
    data->minVal = data->maxVal;

  if (data->val > data->maxVal)
    data->val = data->maxVal;
}
  template <class PrivateData, class Value>
static void setSizeMinimumData(PrivateData *data, const Value &newMinVal)
{
  data->minVal = newMinVal;
  if (data->maxVal.width() < data->minVal.width())
    data->maxVal.setWidth(data->minVal.width());
  if (data->maxVal.height() < data->minVal.height())
    data->maxVal.setHeight(data->minVal.height());

  if (data->val.width() < data->minVal.width())
    data->val.setWidth(data->minVal.width());
  if (data->val.height() < data->minVal.height())
    data->val.setHeight(data->minVal.height());
}

  template <class PrivateData, class Value>
static void setSizeMaximumData(PrivateData *data, const Value &newMaxVal)
{
  data->maxVal = newMaxVal;
  if (data->minVal.width() > data->maxVal.width())
    data->minVal.setWidth(data->maxVal.width());
  if (data->minVal.height() > data->maxVal.height())
    data->minVal.setHeight(data->maxVal.height());

  if (data->val.width() > data->maxVal.width())
    data->val.setWidth(data->maxVal.width());
  if (data->val.height() > data->maxVal.height())
    data->val.setHeight(data->maxVal.height());
}


class QT_QTPROPERTYBROWSER_EXPORT QtGroupPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtGroupPropertyManager(QObject *parent = 0);
    ~QtGroupPropertyManager();

  protected:
    virtual bool hasValue(const QtProperty *property) const;

    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
};

class QtIntPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtIntPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtIntPropertyManager(QObject *parent = 0);
    ~QtIntPropertyManager();

    int value(const QtProperty *property) const;
    int minimum(const QtProperty *property) const;
    int maximum(const QtProperty *property) const;
    int singleStep(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, int val);
    void setMinimum(QtProperty *property, int minVal);
    void setMaximum(QtProperty *property, int maxVal);
    void setRange(QtProperty *property, int minVal, int maxVal);
    void setSingleStep(QtProperty *property, int step);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, int val);
    void rangeChanged(QtProperty *property, int minVal, int maxVal);
    void singleStepChanged(QtProperty *property, int step);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtIntPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtIntPropertyManager)
      Q_DISABLE_COPY(QtIntPropertyManager)
};
class QtIntPropertyManagerPrivate
{
  QtIntPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtIntPropertyManager)
  public:
    QtIntPropertyManagerPrivate() : q_ptr(NULL) {}

    struct Data
    {
      Data() : val(0), minVal(-INT_MAX), maxVal(INT_MAX), singleStep(1) {}
      int val;
      int minVal;
      int maxVal;
      int singleStep;
      int minimumValue() const { return minVal; }
      int maximumValue() const { return maxVal; }
      void setMinimumValue(int newMinVal) { setSimpleMinimumData(this,
          newMinVal); }
      void setMaximumValue(int newMaxVal) { setSimpleMaximumData(this,
          newMaxVal); }
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;
};

class QtBoolPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtBoolPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtBoolPropertyManager(QObject *parent = 0);
    ~QtBoolPropertyManager();

    bool value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, bool val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, bool val);
  protected:
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtBoolPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtBoolPropertyManager)
      Q_DISABLE_COPY(QtBoolPropertyManager)
};

class QtBoolPropertyManagerPrivate
{
  QtBoolPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtBoolPropertyManager)
  public:
    QtBoolPropertyManagerPrivate() :q_ptr(NULL) {}
    QMap<const QtProperty *, bool> m_values;
};


class QtDoublePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtDoublePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtDoublePropertyManager(QObject *parent = 0);
    ~QtDoublePropertyManager();

    double value(const QtProperty *property) const;
    double minimum(const QtProperty *property) const;
    double maximum(const QtProperty *property) const;
    double singleStep(const QtProperty *property) const;
    int decimals(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, double val);
    void setMinimum(QtProperty *property, double minVal);
    void setMaximum(QtProperty *property, double maxVal);
    void setRange(QtProperty *property, double minVal, double maxVal);
    void setSingleStep(QtProperty *property, double step);
    void setDecimals(QtProperty *property, int prec);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, double val);
    void rangeChanged(QtProperty *property, double minVal, double maxVal);
    void singleStepChanged(QtProperty *property, double step);
    void decimalsChanged(QtProperty *property, int prec);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtDoublePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtDoublePropertyManager)
      Q_DISABLE_COPY(QtDoublePropertyManager)
};
class QtDoublePropertyManagerPrivate
{
  QtDoublePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtDoublePropertyManager)
  public:
    QtDoublePropertyManagerPrivate() : q_ptr(NULL) {}
    struct Data
    {
      Data() : val(0), minVal(-INT_MAX), maxVal(INT_MAX), singleStep(1),
      decimals(2) {}
      double val;
      double minVal;
      double maxVal;
      double singleStep;
      int decimals;
      double minimumValue() const { return minVal; }
      double maximumValue() const { return maxVal; }
      void setMinimumValue(double newMinVal) { setSimpleMinimumData(this,
          newMinVal); }
      void setMaximumValue(double newMaxVal) { setSimpleMaximumData(this,
          newMaxVal); }
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;
};
class QtStringPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtStringPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtStringPropertyManager(QObject *parent = 0);
    ~QtStringPropertyManager();

    QString value(const QtProperty *property) const;
    QRegExp regExp(const QtProperty *property) const;
    EchoMode echoMode(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QString &val);
    void setRegExp(QtProperty *property, const QRegExp &regExp);
    void setEchoMode(QtProperty *property, EchoMode echoMode);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QString &val);
    void regExpChanged(QtProperty *property, const QRegExp &regExp);
    void echoModeChanged(QtProperty *property, const int);
  protected:
    QString valueText(const QtProperty *property) const;
    QString displayText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtStringPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtStringPropertyManager)
      Q_DISABLE_COPY(QtStringPropertyManager)
};

class QtStringPropertyManagerPrivate
{
  QtStringPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtStringPropertyManager)
  public:
    QtStringPropertyManagerPrivate() : q_ptr(NULL) {}

    struct Data
    {
      Data() : regExp(QString(QLatin1Char('*')),  Qt::CaseSensitive,
          QRegExp::Wildcard), echoMode(QLineEdit::Normal)
      {
      }
      QString val;
      QRegExp regExp;
      int echoMode;
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    QMap<const QtProperty *, Data> m_values;
};
class QtDatePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtDatePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtDatePropertyManager(QObject *parent = 0);
    ~QtDatePropertyManager();

    QDate value(const QtProperty *property) const;
    QDate minimum(const QtProperty *property) const;
    QDate maximum(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QDate &val);
    void setMinimum(QtProperty *property, const QDate &minVal);
    void setMaximum(QtProperty *property, const QDate &maxVal);
    void setRange(QtProperty *property, const QDate &minVal,
        const QDate &maxVal);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QDate &val);
    void rangeChanged(QtProperty *property, const QDate &minVal,
        const QDate &maxVal);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtDatePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtDatePropertyManager)
      Q_DISABLE_COPY(QtDatePropertyManager)
};
class QtDatePropertyManagerPrivate
{
  QtDatePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtDatePropertyManager)
  public:
    QtDatePropertyManagerPrivate() : q_ptr(NULL) {}
    struct Data
    {
      Data() : val(QDate::currentDate()), minVal(QDate(1752, 9, 14)),
      maxVal(QDate(7999, 12, 31)) {}
      QDate val;
      QDate minVal;
      QDate maxVal;
      QDate minimumValue() const { return minVal; }
      QDate maximumValue() const { return maxVal; }
      void setMinimumValue(const QDate &newMinVal) { setSimpleMinimumData(this,
          newMinVal); }
      void setMaximumValue(const QDate &newMaxVal) { setSimpleMaximumData(this,
          newMaxVal); }
    };

    QString m_format;

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    QMap<const QtProperty *, Data> m_values;
};
class QtTimePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtTimePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtTimePropertyManager(QObject *parent = 0);
    ~QtTimePropertyManager();

    QTime value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QTime &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QTime &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtTimePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtTimePropertyManager)
      Q_DISABLE_COPY(QtTimePropertyManager)
};
class QtTimePropertyManagerPrivate
{
  QtTimePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtTimePropertyManager)
  public:
    QtTimePropertyManagerPrivate() : q_ptr(NULL) {}
    QString m_format;

    typedef QMap<const QtProperty *, QTime> PropertyValueMap;
    PropertyValueMap m_values;
};


class QtDateTimePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtDateTimePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtDateTimePropertyManager(QObject *parent = 0);
    ~QtDateTimePropertyManager();

    QDateTime value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QDateTime &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QDateTime &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtDateTimePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtDateTimePropertyManager)
      Q_DISABLE_COPY(QtDateTimePropertyManager)
};
class QtDateTimePropertyManagerPrivate
{
  QtDateTimePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtDateTimePropertyManager)
  public:
    QtDateTimePropertyManagerPrivate() : q_ptr(NULL) {}
    QString m_format;

    typedef QMap<const QtProperty *, QDateTime> PropertyValueMap;
    PropertyValueMap m_values;
};


class QtKeySequencePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtKeySequencePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtKeySequencePropertyManager(QObject *parent = 0);
    ~QtKeySequencePropertyManager();

    QKeySequence value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QKeySequence &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QKeySequence &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtKeySequencePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtKeySequencePropertyManager)
      Q_DISABLE_COPY(QtKeySequencePropertyManager)
};
class QtKeySequencePropertyManagerPrivate
{
  QtKeySequencePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtKeySequencePropertyManager)
  public:
    QtKeySequencePropertyManagerPrivate() : q_ptr(NULL) {}
    QString m_format;

    typedef QMap<const QtProperty *, QKeySequence> PropertyValueMap;
    PropertyValueMap m_values;
};


class QtCharPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtCharPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtCharPropertyManager(QObject *parent = 0);
    ~QtCharPropertyManager();

    QChar value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QChar &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QChar &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtCharPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtCharPropertyManager)
      Q_DISABLE_COPY(QtCharPropertyManager)
};
class QtCharPropertyManagerPrivate
{
  QtCharPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtCharPropertyManager)
  public:
    QtCharPropertyManagerPrivate() : q_ptr(NULL) {}
    typedef QMap<const QtProperty *, QChar> PropertyValueMap;
    PropertyValueMap m_values;
};


class QtEnumPropertyManager;
class QtLocalePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtLocalePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtLocalePropertyManager(QObject *parent = 0);
    ~QtLocalePropertyManager();

    QtEnumPropertyManager *subEnumPropertyManager() const;

    QLocale value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QLocale &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QLocale &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtLocalePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtLocalePropertyManager)
      Q_DISABLE_COPY(QtLocalePropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotEnumChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtLocalePropertyManagerPrivate
{
  QtLocalePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtLocalePropertyManager)
  public:

    QtLocalePropertyManagerPrivate();

    void slotEnumChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);

    typedef QMap<const QtProperty *, QLocale> PropertyValueMap;
    PropertyValueMap m_values;

    QtEnumPropertyManager *m_enumPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToLanguage;
    QMap<const QtProperty *, QtProperty *> m_propertyToCountry;

    QMap<const QtProperty *, QtProperty *> m_languageToProperty;
    QMap<const QtProperty *, QtProperty *> m_countryToProperty;
};



class QtPointPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtPointPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtPointPropertyManager(QObject *parent = 0);
    ~QtPointPropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;

    QPoint value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QPoint &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QPoint &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtPointPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtPointPropertyManager)
      Q_DISABLE_COPY(QtPointPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtPointPropertyManagerPrivate
{
  QtPointPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtPointPropertyManager)
  public:
    QtPointPropertyManagerPrivate()
      : q_ptr(NULL), m_intPropertyManager(NULL) {}
    void slotIntChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);

    typedef QMap<const QtProperty *, QPoint> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToX;
    QMap<const QtProperty *, QtProperty *> m_propertyToY;

    QMap<const QtProperty *, QtProperty *> m_xToProperty;
    QMap<const QtProperty *, QtProperty *> m_yToProperty;
};

class QtPointFPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtPointFPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtPointFPropertyManager(QObject *parent = 0);
    ~QtPointFPropertyManager();

    QtDoublePropertyManager *subDoublePropertyManager() const;

    QPointF value(const QtProperty *property) const;
    int decimals(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QPointF &val);
    void setDecimals(QtProperty *property, int prec);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QPointF &val);
    void decimalsChanged(QtProperty *property, int prec);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtPointFPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtPointFPropertyManager)
      Q_DISABLE_COPY(QtPointFPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotDoubleChanged(QtProperty *, double))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};
class QtPointFPropertyManagerPrivate
{
  QtPointFPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtPointFPropertyManager)
  public:
    QtPointFPropertyManagerPrivate()
      : q_ptr(NULL), m_doublePropertyManager(NULL) {}
    struct Data
    {
      Data() : decimals(2) {}
      QPointF val;
      int decimals;
    };

    void slotDoubleChanged(QtProperty *property, double value);
    void slotPropertyDestroyed(QtProperty *property);

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtDoublePropertyManager *m_doublePropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToX;
    QMap<const QtProperty *, QtProperty *> m_propertyToY;

    QMap<const QtProperty *, QtProperty *> m_xToProperty;
    QMap<const QtProperty *, QtProperty *> m_yToProperty;
};

class QtSizePropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtSizePropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtSizePropertyManager(QObject *parent = 0);
    ~QtSizePropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;

    QSize value(const QtProperty *property) const;
    QSize minimum(const QtProperty *property) const;
    QSize maximum(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QSize &val);
    void setMinimum(QtProperty *property, const QSize &minVal);
    void setMaximum(QtProperty *property, const QSize &maxVal);
    void setRange(QtProperty *property, const QSize &minVal,
        const QSize &maxVal);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QSize &val);
    void rangeChanged(QtProperty *property, const QSize &minVal,
        const QSize &maxVal);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtSizePropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtSizePropertyManager)
      Q_DISABLE_COPY(QtSizePropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtSizePropertyManagerPrivate
{
  QtSizePropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtSizePropertyManager)
  public:
    QtSizePropertyManagerPrivate()
      : q_ptr(NULL), m_intPropertyManager(NULL) {}
    void slotIntChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);
    void setValue(QtProperty *property, const QSize &val);
    void setRange(QtProperty *property,
        const QSize &minVal, const QSize &maxVal, const QSize &val);

    struct Data
    {
      Data() : val(QSize(0, 0)), minVal(QSize(0, 0)), maxVal(QSize(INT_MAX,
            INT_MAX)) {}
      QSize val;
      QSize minVal;
      QSize maxVal;
      QSize minimumValue() const { return minVal; }
      QSize maximumValue() const { return maxVal; }
      void setMinimumValue(const QSize &newMinVal) { setSizeMinimumData(this,
          newMinVal); }
      void setMaximumValue(const QSize &newMaxVal) { setSizeMaximumData(this,
          newMaxVal); }
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToW;
    QMap<const QtProperty *, QtProperty *> m_propertyToH;

    QMap<const QtProperty *, QtProperty *> m_wToProperty;
    QMap<const QtProperty *, QtProperty *> m_hToProperty;
};

class QtSizeFPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtSizeFPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtSizeFPropertyManager(QObject *parent = 0);
    ~QtSizeFPropertyManager();

    QtDoublePropertyManager *subDoublePropertyManager() const;

    QSizeF value(const QtProperty *property) const;
    QSizeF minimum(const QtProperty *property) const;
    QSizeF maximum(const QtProperty *property) const;
    int decimals(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QSizeF &val);
    void setMinimum(QtProperty *property, const QSizeF &minVal);
    void setMaximum(QtProperty *property, const QSizeF &maxVal);
    void setRange(QtProperty *property, const QSizeF &minVal,
        const QSizeF &maxVal);
    void setDecimals(QtProperty *property, int prec);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QSizeF &val);
    void rangeChanged(QtProperty *property, const QSizeF &minVal,
        const QSizeF &maxVal);
    void decimalsChanged(QtProperty *property, int prec);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtSizeFPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtSizeFPropertyManager)
      Q_DISABLE_COPY(QtSizeFPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotDoubleChanged(QtProperty *, double))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtSizeFPropertyManagerPrivate
{
  QtSizeFPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtSizeFPropertyManager)
  public:
    QtSizeFPropertyManagerPrivate()
      : q_ptr(NULL), m_doublePropertyManager(NULL) {}
    void slotDoubleChanged(QtProperty *property, double value);
    void slotPropertyDestroyed(QtProperty *property);
    void setValue(QtProperty *property, const QSizeF &val);
    void setRange(QtProperty *property,
        const QSizeF &minVal, const QSizeF &maxVal, const QSizeF &val);

    struct Data
    {
      Data() : val(QSizeF(0, 0)), minVal(QSizeF(0, 0)), maxVal(QSizeF(INT_MAX,
            INT_MAX)), decimals(2) {}
      QSizeF val;
      QSizeF minVal;
      QSizeF maxVal;
      int decimals;
      QSizeF minimumValue() const { return minVal; }
      QSizeF maximumValue() const { return maxVal; }
      void setMinimumValue(const QSizeF &newMinVal) { setSizeMinimumData(this,
          newMinVal); }
      void setMaximumValue(const QSizeF &newMaxVal) { setSizeMaximumData(this,
          newMaxVal); }
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtDoublePropertyManager *m_doublePropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToW;
    QMap<const QtProperty *, QtProperty *> m_propertyToH;

    QMap<const QtProperty *, QtProperty *> m_wToProperty;
    QMap<const QtProperty *, QtProperty *> m_hToProperty;
};

class QtRectPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtRectPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtRectPropertyManager(QObject *parent = 0);
    ~QtRectPropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;

    QRect value(const QtProperty *property) const;
    QRect constraint(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QRect &val);
    void setConstraint(QtProperty *property, const QRect &constraint);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QRect &val);
    void constraintChanged(QtProperty *property, const QRect &constraint);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtRectPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtRectPropertyManager)
      Q_DISABLE_COPY(QtRectPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtRectPropertyManagerPrivate
{
  QtRectPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtRectPropertyManager)
  public:
    QtRectPropertyManagerPrivate()
      : q_ptr(NULL), m_intPropertyManager(NULL) {}

    void slotIntChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);
    void setConstraint(QtProperty *property, const QRect &constraint,
        const QRect &val);

    struct Data
    {
      Data() : val(0, 0, 0, 0) {}
      QRect val;
      QRect constraint;
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToX;
    QMap<const QtProperty *, QtProperty *> m_propertyToY;
    QMap<const QtProperty *, QtProperty *> m_propertyToW;
    QMap<const QtProperty *, QtProperty *> m_propertyToH;

    QMap<const QtProperty *, QtProperty *> m_xToProperty;
    QMap<const QtProperty *, QtProperty *> m_yToProperty;
    QMap<const QtProperty *, QtProperty *> m_wToProperty;
    QMap<const QtProperty *, QtProperty *> m_hToProperty;
};

class QtRectFPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtRectFPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtRectFPropertyManager(QObject *parent = 0);
    ~QtRectFPropertyManager();

    QtDoublePropertyManager *subDoublePropertyManager() const;

    QRectF value(const QtProperty *property) const;
    QRectF constraint(const QtProperty *property) const;
    int decimals(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QRectF &val);
    void setConstraint(QtProperty *property, const QRectF &constraint);
    void setDecimals(QtProperty *property, int prec);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QRectF &val);
    void constraintChanged(QtProperty *property, const QRectF &constraint);
    void decimalsChanged(QtProperty *property, int prec);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtRectFPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtRectFPropertyManager)
      Q_DISABLE_COPY(QtRectFPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotDoubleChanged(QtProperty *, double))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtRectFPropertyManagerPrivate
{
  QtRectFPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtRectFPropertyManager)
  public:
    QtRectFPropertyManagerPrivate()
      : q_ptr(NULL), m_doublePropertyManager(NULL) {}
    void slotDoubleChanged(QtProperty *property, double value);
    void slotPropertyDestroyed(QtProperty *property);
    void setConstraint(QtProperty *property, const QRectF &constraint,
        const QRectF &val);

    struct Data
    {
      Data() : val(0, 0, 0, 0), decimals(2) {}
      QRectF val;
      QRectF constraint;
      int decimals;
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtDoublePropertyManager *m_doublePropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToX;
    QMap<const QtProperty *, QtProperty *> m_propertyToY;
    QMap<const QtProperty *, QtProperty *> m_propertyToW;
    QMap<const QtProperty *, QtProperty *> m_propertyToH;

    QMap<const QtProperty *, QtProperty *> m_xToProperty;
    QMap<const QtProperty *, QtProperty *> m_yToProperty;
    QMap<const QtProperty *, QtProperty *> m_wToProperty;
    QMap<const QtProperty *, QtProperty *> m_hToProperty;
};

class QtEnumPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtEnumPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtEnumPropertyManager(QObject *parent = 0);
    ~QtEnumPropertyManager();

    int value(const QtProperty *property) const;
    QStringList enumNames(const QtProperty *property) const;
    QMap<int, QIcon> enumIcons(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, int val);
    void setEnumNames(QtProperty *property, const QStringList &names);
    void setEnumIcons(QtProperty *property, const QMap<int, QIcon> &icons);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, int val);
    void enumNamesChanged(QtProperty *property, const QStringList &names);
    void enumIconsChanged(QtProperty *property, const QMap<int, QIcon> &icons);
  protected:
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtEnumPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtEnumPropertyManager)
      Q_DISABLE_COPY(QtEnumPropertyManager)
};

class QtEnumPropertyManagerPrivate
{
  QtEnumPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtEnumPropertyManager)
  public:
    QtEnumPropertyManagerPrivate() : q_ptr(NULL) {}

    struct Data
    {
      Data() : val(-1) {}
      int val;
      QStringList enumNames;
      QMap<int, QIcon> enumIcons;
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;
};

class QtFlagPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtFlagPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtFlagPropertyManager(QObject *parent = 0);
    ~QtFlagPropertyManager();

    QtBoolPropertyManager *subBoolPropertyManager() const;

    int value(const QtProperty *property) const;
    QStringList flagNames(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, int val);
    void setFlagNames(QtProperty *property, const QStringList &names);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, int val);
    void flagNamesChanged(QtProperty *property, const QStringList &names);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtFlagPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtFlagPropertyManager)
      Q_DISABLE_COPY(QtFlagPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotBoolChanged(QtProperty *, bool))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtFlagPropertyManagerPrivate
{
  QtFlagPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtFlagPropertyManager)
  public:
    QtFlagPropertyManagerPrivate()
      : q_ptr(NULL), m_boolPropertyManager(NULL) {}
    void slotBoolChanged(QtProperty *property, bool value);
    void slotPropertyDestroyed(QtProperty *property);

    struct Data
    {
      Data() : val(-1) {}
      int val;
      QStringList flagNames;
    };

    typedef QMap<const QtProperty *, Data> PropertyValueMap;
    PropertyValueMap m_values;

    QtBoolPropertyManager *m_boolPropertyManager;

    QMap<const QtProperty *, QList<QtProperty *> > m_propertyToFlags;

    QMap<const QtProperty *, QtProperty *> m_flagToProperty;
};

class QtSizePolicyPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtSizePolicyPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtSizePolicyPropertyManager(QObject *parent = 0);
    ~QtSizePolicyPropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;
    QtEnumPropertyManager *subEnumPropertyManager() const;

    QSizePolicy value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QSizePolicy &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QSizePolicy &val);
  protected:
    QString valueText(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtSizePolicyPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtSizePolicyPropertyManager)
      Q_DISABLE_COPY(QtSizePolicyPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotEnumChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtSizePolicyPropertyManagerPrivate
{
  QtSizePolicyPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtSizePolicyPropertyManager)
  public:

    QtSizePolicyPropertyManagerPrivate();

    void slotIntChanged(QtProperty *property, int value);
    void slotEnumChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);

    typedef QMap<const QtProperty *, QSizePolicy> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;
    QtEnumPropertyManager *m_enumPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToHPolicy;
    QMap<const QtProperty *, QtProperty *> m_propertyToVPolicy;
    QMap<const QtProperty *, QtProperty *> m_propertyToHStretch;
    QMap<const QtProperty *, QtProperty *> m_propertyToVStretch;

    QMap<const QtProperty *, QtProperty *> m_hPolicyToProperty;
    QMap<const QtProperty *, QtProperty *> m_vPolicyToProperty;
    QMap<const QtProperty *, QtProperty *> m_hStretchToProperty;
    QMap<const QtProperty *, QtProperty *> m_vStretchToProperty;
};

class QtFontPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtFontPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtFontPropertyManager(QObject *parent = 0);
    ~QtFontPropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;
    QtEnumPropertyManager *subEnumPropertyManager() const;
    QtBoolPropertyManager *subBoolPropertyManager() const;

    QFont value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QFont &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QFont &val);
  protected:
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtFontPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtFontPropertyManager)
      Q_DISABLE_COPY(QtFontPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotEnumChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotBoolChanged(QtProperty *, bool))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
      Q_PRIVATE_SLOT(d_func(), void slotFontDatabaseChanged())
      Q_PRIVATE_SLOT(d_func(), void slotFontDatabaseDelayedChange())
};

class QtFontPropertyManagerPrivate
{
  QtFontPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtFontPropertyManager)
  public:

    QtFontPropertyManagerPrivate();

    void slotIntChanged(QtProperty *property, int value);
    void slotEnumChanged(QtProperty *property, int value);
    void slotBoolChanged(QtProperty *property, bool value);
    void slotPropertyDestroyed(QtProperty *property);
    void slotFontDatabaseChanged();
    void slotFontDatabaseDelayedChange();

    QStringList m_familyNames;

    typedef QMap<const QtProperty *, QFont> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;
    QtEnumPropertyManager *m_enumPropertyManager;
    QtBoolPropertyManager *m_boolPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToFamily;
    QMap<const QtProperty *, QtProperty *> m_propertyToPointSize;
    QMap<const QtProperty *, QtProperty *> m_propertyToBold;
    QMap<const QtProperty *, QtProperty *> m_propertyToItalic;
    QMap<const QtProperty *, QtProperty *> m_propertyToUnderline;
    QMap<const QtProperty *, QtProperty *> m_propertyToStrikeOut;
    QMap<const QtProperty *, QtProperty *> m_propertyToKerning;

    QMap<const QtProperty *, QtProperty *> m_familyToProperty;
    QMap<const QtProperty *, QtProperty *> m_pointSizeToProperty;
    QMap<const QtProperty *, QtProperty *> m_boldToProperty;
    QMap<const QtProperty *, QtProperty *> m_italicToProperty;
    QMap<const QtProperty *, QtProperty *> m_underlineToProperty;
    QMap<const QtProperty *, QtProperty *> m_strikeOutToProperty;
    QMap<const QtProperty *, QtProperty *> m_kerningToProperty;

    bool m_settingValue;
    QTimer *m_fontDatabaseChangeTimer;
};

class QtColorPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtColorPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtColorPropertyManager(QObject *parent = 0);
    ~QtColorPropertyManager();

    QtIntPropertyManager *subIntPropertyManager() const;

    QColor value(const QtProperty *property) const;

    public Q_SLOTS:
      void setValue(QtProperty *property, const QColor &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QColor &val);
  protected:
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtColorPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtColorPropertyManager)
      Q_DISABLE_COPY(QtColorPropertyManager)
      Q_PRIVATE_SLOT(d_func(), void slotIntChanged(QtProperty *, int))
      Q_PRIVATE_SLOT(d_func(), void slotPropertyDestroyed(QtProperty *))
};

class QtColorPropertyManagerPrivate
{
  QtColorPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtColorPropertyManager)
  public:
    QtColorPropertyManagerPrivate()
      : q_ptr(NULL), m_intPropertyManager(NULL) {}
    void slotIntChanged(QtProperty *property, int value);
    void slotPropertyDestroyed(QtProperty *property);

    typedef QMap<const QtProperty *, QColor> PropertyValueMap;
    PropertyValueMap m_values;

    QtIntPropertyManager *m_intPropertyManager;

    QMap<const QtProperty *, QtProperty *> m_propertyToR;
    QMap<const QtProperty *, QtProperty *> m_propertyToG;
    QMap<const QtProperty *, QtProperty *> m_propertyToB;
    QMap<const QtProperty *, QtProperty *> m_propertyToA;

    QMap<const QtProperty *, QtProperty *> m_rToProperty;
    QMap<const QtProperty *, QtProperty *> m_gToProperty;
    QMap<const QtProperty *, QtProperty *> m_bToProperty;
    QMap<const QtProperty *, QtProperty *> m_aToProperty;
};

class QtCursorPropertyManagerPrivate;


class QT_QTPROPERTYBROWSER_EXPORT QtCursorPropertyManager
: public QtAbstractPropertyManager
{
  Q_OBJECT
  public:
    explicit QtCursorPropertyManager(QObject *parent = 0);
    ~QtCursorPropertyManager();

#ifndef QT_NO_CURSOR
    QCursor value(const QtProperty *property) const;
#endif

    public Q_SLOTS:
      void setValue(QtProperty *property, const QCursor &val);
  Q_SIGNALS:
    void valueChanged(QtProperty *property, const QCursor &val);
  protected:
    QString valueText(const QtProperty *property) const;
    QIcon valueIcon(const QtProperty *property) const;
    virtual void initializeProperty(QtProperty *property);
    virtual void uninitializeProperty(QtProperty *property);
  private:
    QtCursorPropertyManagerPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QtCursorPropertyManager)
      Q_DISABLE_COPY(QtCursorPropertyManager)
};

class QtCursorPropertyManagerPrivate
{
  QtCursorPropertyManager *q_ptr;
  Q_DECLARE_PUBLIC(QtCursorPropertyManager)
  public:
    QtCursorPropertyManagerPrivate() : q_ptr(NULL) {}
    typedef QMap<const QtProperty *, QCursor> PropertyValueMap;
    PropertyValueMap m_values;
};

class QtMetaEnumProvider
{
  public:
    QtMetaEnumProvider();

    QStringList policyEnumNames() const { return m_policyEnumNames; }
    QStringList languageEnumNames() const { return m_languageEnumNames; }
    QStringList countryEnumNames(QLocale::Language language) const
    { return m_countryEnumNames.value(language); }

    QSizePolicy::Policy indexToSizePolicy(int index) const;
    int sizePolicyToIndex(QSizePolicy::Policy policy) const;

    void indexToLocale(int languageIndex, int countryIndex,
        QLocale::Language *language, QLocale::Country *country) const;
    void localeToIndex(QLocale::Language language, QLocale::Country country,
        int *languageIndex, int *countryIndex) const;

  private:
    void initLocale();

    QStringList m_policyEnumNames;
    QStringList m_languageEnumNames;
    QMap<QLocale::Language, QStringList> m_countryEnumNames;
    QMap<int, QLocale::Language> m_indexToLanguage;
    QMap<QLocale::Language, int> m_languageToIndex;
    QMap<int, QMap<int, QLocale::Country> > m_indexToCountry;
    QMap<QLocale::Language, QMap<QLocale::Country, int> > m_countryToIndex;
    QMetaEnum m_policyEnum;
};

class QtMetaEnumWrapper
: public QObject
{
  Q_OBJECT
    Q_PROPERTY(QSizePolicy::Policy policy READ policy)
  public:
    QSizePolicy::Policy policy() const { return QSizePolicy::Ignored; }
  private:
    explicit QtMetaEnumWrapper(QObject *parent) : QObject(parent) {}
};


#if QT_VERSION >= 0x040400
QT_END_NAMESPACE
#endif

#endif
