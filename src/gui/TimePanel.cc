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

  this->pauseLabel = new QLabel(tr("<font style='color:'#dddddd'>Paused</font>"));

  QLabel *percentRealTimeLabel = new QLabel(tr("Real Time Factor:"));
  QLabel *simTimeLabel = new QLabel(tr("Sim Time:"));
  QLabel *realTimeLabel = new QLabel(tr("Real Time:"));

  mainLayout->addWidget(percentRealTimeLabel);
  mainLayout->addWidget(this->percentRealTimeEdit);

  mainLayout->addWidget(simTimeLabel);
  mainLayout->addWidget(this->simTimeEdit);

  mainLayout->addWidget(realTimeLabel);
  mainLayout->addWidget(this->realTimeEdit);

  mainLayout->addWidget(this->pauseLabel);
  mainLayout->addItem(new QSpacerItem(20,20,QSizePolicy::Expanding, QSizePolicy::Minimum));

  this->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->statsSub = this->node->Subscribe("~/world_stats", &TimePanel::OnStats, this);

  QTimer *timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(Update()) );
  timer->start(33);

  this->simTime.Set(0);
}

TimePanel::~TimePanel()
{
}

void TimePanel::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->simTime  = msgs::Convert( _msg->sim_time() );
  this->realTime = msgs::Convert( _msg->real_time() );
  if (_msg->paused())
    this->pauseLabel->setText("<font style='color:green;font-weight:bold;'>Paused</font>");
  else
    this->pauseLabel->setText("<font style='color:green;font-weight:bold;'>      </font>");
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

  if (simDbl > 0)
    percent << std::fixed << std::setprecision(2) << ( this->simTime / this->realTime).Double();
  else
    percent << "0";

  this->percentRealTimeEdit->setText( tr(percent.str().c_str()) );

  this->simTimeEdit->setText( tr(sim.str().c_str()));
  this->realTimeEdit->setText( tr(real.str().c_str()) );
}
