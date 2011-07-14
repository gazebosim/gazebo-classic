#include <QtGui>

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/EditSceneWidget.hh"

using namespace gazebo;
using namespace gui;

EditSceneWidget::EditSceneWidget( QWidget *parent )
  : QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *ambientLayout = new QHBoxLayout;
  this->ambientColorButton = new QPushButton("ambient");
  this->ambientColorFrame = new QFrame();
  this->ambientColorFrame->setLineWidth(1);
  this->ambientColorFrame->setFrameShadow(QFrame::Sunken);
  this->ambientColorFrame->setFrameShape(QFrame::Box);
  ambientLayout->addWidget(this->ambientColorButton);
  ambientLayout->addWidget(this->ambientColorFrame);

  QHBoxLayout *backgroundLayout = new QHBoxLayout;
  this->backgroundColorButton = new QPushButton("background");
  this->backgroundColorFrame = new QFrame();
  this->backgroundColorFrame->setLineWidth(1);
  this->backgroundColorFrame->setFrameShadow(QFrame::Sunken);
  this->backgroundColorFrame->setFrameShape(QFrame::Box);
  backgroundLayout->addWidget(this->backgroundColorButton);
  backgroundLayout->addWidget(this->backgroundColorFrame);

  this->shadowsButton = new QCheckBox("Shadows");
  this->shadowsButton->setCheckState( Qt::Checked );

  mainLayout->addLayout(ambientLayout);
  mainLayout->addLayout(backgroundLayout);
  mainLayout->addWidget(shadowsButton);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");

  connect( this->ambientColorButton, SIGNAL(clicked()), 
           this, SLOT(AmbientColor()) );
  connect( this->backgroundColorButton, SIGNAL(clicked()), 
           this, SLOT(BackgroundColor()) );
  connect( this->shadowsButton, SIGNAL(toggled(bool)), 
           this, SLOT(Shadows(bool)) );
}

EditSceneWidget::~EditSceneWidget()
{
}

void EditSceneWidget::AmbientColor()
{
  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  QPalette palette = this->ambientColorFrame->palette();
  palette.setColor( backgroundRole(), color );
  this->ambientColorFrame->setPalette( palette );
  this->ambientColorFrame->setAutoFillBackground( true );

  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_ambient()->set_r( color.red() /256.0 );
  msg.mutable_ambient()->set_g( color.green() /256.0 );
  msg.mutable_ambient()->set_b( color.blue() /256.0 );
  msg.mutable_ambient()->set_a( 1.0 );
  this->scenePub->Publish(msg);
}

void EditSceneWidget::BackgroundColor()
{
  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  QPalette palette = this->backgroundColorFrame->palette();
  palette.setColor( backgroundRole(), color );
  this->backgroundColorFrame->setPalette( palette );
  this->backgroundColorFrame->setAutoFillBackground( true );

  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_background()->set_r( color.red() / 256.0 );
  msg.mutable_background()->set_g( color.green() / 256.0 );
  msg.mutable_background()->set_b( color.blue() / 256.0 );
  msg.mutable_background()->set_a( 1.0 );
  this->scenePub->Publish(msg);
}

void EditSceneWidget::Shadows(bool _state)
{
  std::cout << "Shadows[" << _state << "]\n";

  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.set_shadows( _state );
  this->scenePub->Publish(msg);
}

void EditSceneWidget::Init()
{
}
