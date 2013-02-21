#pragma GCC system_header
#pragma GCC diagnostic ignored "-Wswitch-default"
/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_COUNTER_H
#define QWT_COUNTER_H

#include "qwt_global.h"
#include "qwt_double_range.h"
#include <qwidget.h>

/*!
  \brief The Counter Widget

  A Counter consists of a lineEdit displaying a number and
  one ore more (up to three) push buttons on each side
  of the lineEdit which can be used to increment or decrement
  the counter's value.

  A Counter has a range from a minimum value to a maximum value
  and a step size. The range can be specified using setRange().
  The number of steps by which a button increments or decrements
  the value can be specified using setIncSteps().
  The number of buttons can be changed with setNumButtons().

  Example:
\code
#include <qwt_counter.h>

QwtCounter *counter = new QwtCounter(parent);

counter->setRange(0.0, 100.0, 1.0);             // From 0.0 to 100, step 1.0
counter->setNumButtons(2);                      // Two buttons each side
counter->setIncSteps(QwtCounter::Button1, 1);   // Button 1 increments 1 step
counter->setIncSteps(QwtCounter::Button2, 20);  // Button 2 increments 20 steps

connect(counter, SIGNAL(valueChanged(double)), my_class, SLOT(newValue(double)));
\endcode
 */

class QWT_EXPORT QwtCounter : public QWidget, public QwtDoubleRange
{
    Q_OBJECT

    Q_PROPERTY( int numButtons READ numButtons WRITE setNumButtons )
    Q_PROPERTY( double basicstep READ step WRITE setStep )
    Q_PROPERTY( double minValue READ minValue WRITE setMinValue )
    Q_PROPERTY( double maxValue READ maxValue WRITE setMaxValue )
    Q_PROPERTY( int stepButton1 READ stepButton1 WRITE setStepButton1 )
    Q_PROPERTY( int stepButton2 READ stepButton2 WRITE setStepButton2 )
    Q_PROPERTY( int stepButton3 READ stepButton3 WRITE setStepButton3 )
    Q_PROPERTY( double value READ value WRITE setValue )
    Q_PROPERTY( bool editable READ editable WRITE setEditable )

public:
    //! Button index
    enum Button
    {
        //! Button intended for minor steps
        Button1,

        //! Button intended for medium steps
        Button2,

        //! Button intended for large steps
        Button3,

        //! Number of buttons
        ButtonCnt
    };

    explicit QwtCounter( QWidget *parent = NULL );
    virtual ~QwtCounter();

    bool editable() const;
    void setEditable( bool );

    void setNumButtons( int n );
    int numButtons() const;

    void setIncSteps( QwtCounter::Button btn, int nSteps );
    int incSteps( QwtCounter::Button btn ) const;

    virtual void setValue( double );
    virtual QSize sizeHint() const;

    // a set of dummies to help the designer

    double step() const;
    void setStep( double s );

    double minValue() const;
    void setMinValue( double m );

    double maxValue() const;
    void setMaxValue( double m );

    void setStepButton1( int nSteps );
    int stepButton1() const;

    void setStepButton2( int nSteps );
    int stepButton2() const;

    void setStepButton3( int nSteps );
    int stepButton3() const;

    virtual double value() const;

Q_SIGNALS:
    /*!
        This signal is emitted when a button has been released
        \param value The new value
    */
    void buttonReleased ( double value );

    /*!
        This signal is emitted when the counter's value has changed
        \param value The new value
    */
    void valueChanged ( double value );

protected:
    virtual bool event( QEvent * );
    virtual void wheelEvent( QWheelEvent * );
    virtual void keyPressEvent( QKeyEvent * );
    virtual void rangeChange();

private Q_SLOTS:
    void btnReleased();
    void btnClicked();
    void textChanged();

private:
    void initCounter();
    void updateButtons();
    void showNum( double );
    virtual void valueChange();

    class PrivateData;
    PrivateData *d_data;
};

#endif
