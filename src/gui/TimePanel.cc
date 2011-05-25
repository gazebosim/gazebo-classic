#include <QtGui>
#include <sstream>

#include "transport/Node.hh"
#include "gui/TimePanel.hh"

using namespace gazebo;
using namespace gui;

TimePanel::TimePanel( QWidget *parent )
  : QWidget( parent )
{
  QHBoxLayout *mainLayout = new QHBoxLayout;

  this->percentRealTimeEdit = new QLineEdit;
  this->percentRealTimeEdit->setReadOnly(true);
  this->percentRealTimeEdit->setFixedWidth(90);

  this->simTimeEdit = new QLineEdit;
  this->simTimeEdit->setReadOnly(true);
  this->simTimeEdit->setFixedWidth(110);

  this->realTimeEdit = new QLineEdit;
  this->realTimeEdit->setReadOnly(true);
  this->realTimeEdit->setFixedWidth(110);

  this->pauseTimeEdit = new QLineEdit;
  this->pauseTimeEdit->setReadOnly(true);
  this->pauseTimeEdit->setFixedWidth(90);

  QLabel *percentRealTimeLabel = new QLabel(tr("Real Time Factor:"));
  QLabel *simTimeLabel = new QLabel(tr("Sim Time:"));
  QLabel *realTimeLabel = new QLabel(tr("Real Time:"));
  QLabel *pauseTimeLabel = new QLabel(tr("Pause Time:"));

  mainLayout->addWidget(percentRealTimeLabel);
  mainLayout->addWidget(this->percentRealTimeEdit);

  mainLayout->addWidget(simTimeLabel);
  mainLayout->addWidget(this->simTimeEdit);

  mainLayout->addWidget(realTimeLabel);
  mainLayout->addWidget(this->realTimeEdit);

  mainLayout->addWidget(pauseTimeLabel);
  mainLayout->addWidget(this->pauseTimeEdit);
  mainLayout->addItem(new QSpacerItem(20,20,QSizePolicy::Expanding, QSizePolicy::Minimum));

  this->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->statsSub = this->node->Subscribe("~/world_stats", &TimePanel::OnStats, this);

  QTimer *timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(Update()) );
  timer->start(33);
}

TimePanel::~TimePanel()
{
}

void TimePanel::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &msg)
{
  this->simTime  = common::Message::Convert( msg->sim_time() );
  this->realTime = common::Message::Convert( msg->real_time() );
  this->pauseTime = common::Message::Convert( msg->pause_time() );
}

void TimePanel::Update()
{
  std::ostringstream percent;
  std::ostringstream sim;
  std::ostringstream real;
  std::ostringstream pause;

  double simDbl = this->simTime.Double();
  if (simDbl > 31536000)
    sim << std::fixed << std::setprecision(2) << simDbl/31536000 << " dys";
  else if (simDbl > 86400)
    sim << std::fixed << std::setprecision(2) << simDbl / 86400 << " dys";
  else if (simDbl > 3600)
    sim << std::fixed << std::setprecision(2) << simDbl/3600 << " hrs";
  else if (simDbl > 999)
    sim << std::fixed << std::setprecision(2) << simDbl/60 << " min";
  else
    sim << std::fixed << std::setprecision(2) << simDbl << " sec";

  double realDbl = this->realTime.Double();
  if (realDbl > 31536000)
    real << std::fixed << std::setprecision(2) << realDbl/31536000 << " dys";
  else if (realDbl > 86400)
    real << std::fixed << std::setprecision(2) << realDbl/86400 << " dys";
  else if (realDbl > 3600)
    real << std::fixed << std::setprecision(2) << realDbl/3600 << " hrs";
  else if (realDbl > 999)
    real << std::fixed << std::setprecision(2) << realDbl/60 << " min";
  else
    real << std::fixed << std::setprecision(2) << realDbl << " sec";

  percent << std::fixed << std::setprecision(2) << ( this->simTime / this->realTime).Double();
  this->percentRealTimeEdit->setText( tr(percent.str().c_str()) );

  this->simTimeEdit->setText( tr(sim.str().c_str()));
  this->realTimeEdit->setText( tr(real.str().c_str()) );

  pause << std::fixed << std::setprecision(2) << pauseTime.Double();
  this->pauseTimeEdit->setText( tr(pause.str().c_str()) );
}


