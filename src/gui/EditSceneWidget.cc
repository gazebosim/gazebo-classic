#include <QtGui>

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/EditSceneWidget.hh"

using namespace gazebo;
using namespace gui;

EditSceneWidget::EditSceneWidget( QWidget *parent )
  : QWidget(parent)
{
  this->setWindowTitle("Gazebo: Scene");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QGroupBox *ambientBox = new QGroupBox(tr("Ambient"));
  ambientBox->setStyleSheet(tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QHBoxLayout *ambientLayout = new QHBoxLayout;
  this->ambientColorButton = new QPushButton;
  QLabel *ambientColorLabel = new QLabel(tr("Color:"));
  ambientLayout->addWidget(ambientColorLabel);
  ambientLayout->addWidget(this->ambientColorButton);
  ambientLayout->insertStretch(1,0);
  ambientBox->setLayout(ambientLayout);


  QGroupBox *backgroundBox = new QGroupBox(tr("Background"));
  backgroundBox->setStyleSheet(tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QHBoxLayout *backgroundLayout = new QHBoxLayout;
  this->backgroundColorButton = new QPushButton;
  QLabel *backgroundColorLabel = new QLabel(tr("Color:"));
  backgroundLayout->addWidget(backgroundColorLabel);
  backgroundLayout->addWidget(this->backgroundColorButton);
  backgroundLayout->insertStretch(1,0);
  backgroundBox->setLayout(backgroundLayout);

  QGroupBox *shadowsBox = new QGroupBox(tr("Shadows"));
  shadowsBox->setStyleSheet(tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QVBoxLayout *shadowsBoxLayout = new QVBoxLayout;
  this->shadowsButton = new QCheckBox(tr("Enabled"));
  this->shadowsButton->setCheckState( Qt::Checked );
  shadowsBoxLayout->addWidget(this->shadowsButton);
  shadowsBox->setLayout(shadowsBoxLayout);

  QGroupBox *fogBox = new QGroupBox(tr("Fog"));
  fogBox->setCheckable(true);
  fogBox->setStyleSheet(tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));

  QVBoxLayout *fogBoxLayout = new QVBoxLayout;

  QHBoxLayout *fogTypeLayout = new QHBoxLayout;
  QComboBox *fogTypeBox = new QComboBox();
  fogTypeBox->addItem(tr("Linear"));
  fogTypeBox->addItem(tr("Exponential"));
  fogTypeBox->addItem(tr("Squared"));
  QLabel *fogTypeLabel = new QLabel(tr("Type:"));
  fogTypeLayout->addWidget( fogTypeLabel );
  fogTypeLayout->insertStretch(1,0);
  fogTypeLayout->addWidget( fogTypeBox );

  QHBoxLayout *fogColorLayout = new QHBoxLayout;
  this->fogColorButton = new QPushButton;
  QLabel *fogColorLabel = new QLabel(tr("Color:"));
  fogColorLayout->addWidget(fogColorLabel);
  fogColorLayout->addWidget(this->fogColorButton);
  fogColorLayout->insertStretch(1,0);

  QHBoxLayout *fogStartLayout = new QHBoxLayout;
  this->fogStart = new QLineEdit;
  this->fogStart->setValidator(new QDoubleValidator(this->fogStart) );
  this->fogStart->setInputMethodHints(Qt::ImhDigitsOnly);
  this->fogStart->setFixedWidth(80);
  QLabel *fogStartLabel = new QLabel(tr("Start:"));
  fogStartLayout->addWidget(fogStartLabel);
  fogStartLayout->addWidget(fogStart);
  fogStartLayout->insertStretch(1,0);

  QHBoxLayout *fogEndLayout = new QHBoxLayout;
  this->fogEnd = new QLineEdit;
  this->fogEnd->setValidator(new QDoubleValidator(this->fogEnd) );
  this->fogEnd->setInputMethodHints(Qt::ImhDigitsOnly);
  this->fogEnd->setFixedWidth(80);
  QLabel *fogEndLabel = new QLabel(tr("End:"));
  fogEndLayout->addWidget(fogEndLabel);
  fogEndLayout->addWidget(fogEnd);
  fogEndLayout->insertStretch(1,0);


  QHBoxLayout *fogDensityLayout = new QHBoxLayout;
  this->fogDensitySpin = new QDoubleSpinBox;
  this->fogDensitySpin->setRange(0.0,1.0);
  this->fogDensitySpin->setSingleStep(0.01);
  QLabel *fogDensityLabel = new QLabel(tr("Density:"));
  fogDensityLayout->addWidget(fogDensityLabel);
  fogDensityLayout->addItem(new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum));
  fogDensityLayout->addWidget(this->fogDensitySpin);
  fogDensityLayout->insertStretch(1,0);


  fogBoxLayout->addLayout(fogColorLayout);
  fogBoxLayout->addLayout(fogTypeLayout);
  fogBoxLayout->addLayout(fogStartLayout);
  fogBoxLayout->addLayout(fogEndLayout);
  fogBoxLayout->addLayout(fogDensityLayout);


  fogBox->setLayout(fogBoxLayout);


  mainLayout->addWidget(shadowsBox);
  mainLayout->addWidget(ambientBox);
  mainLayout->addWidget(backgroundBox);
  mainLayout->addWidget(fogBox);
  mainLayout->insertStretch(4,0);

  this->setLayout(mainLayout);
  this->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");

  connect( this->ambientColorButton, SIGNAL(clicked()), 
           this, SLOT(AmbientColor()) );
  connect( this->backgroundColorButton, SIGNAL(clicked()), 
           this, SLOT(BackgroundColor()) );
  connect( this->fogColorButton, SIGNAL(clicked()), 
           this, SLOT(FogColor()) );

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

  std::ostringstream style;
  style << "background-color:rgb(" << color.red() << "," << color.green() << "," << color.blue() << ");";
  this->ambientColorButton->setAutoFillBackground( true );
  this->ambientColorButton->setStyleSheet(style.str().c_str());

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

  std::ostringstream style;
  style << "background-color:rgb(" << color.red() << "," << color.green() << "," << color.blue() << ");";
  this->backgroundColorButton->setAutoFillBackground( true );
  this->backgroundColorButton->setStyleSheet(style.str().c_str());


  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_background()->set_r( color.red() / 256.0 );
  msg.mutable_background()->set_g( color.green() / 256.0 );
  msg.mutable_background()->set_b( color.blue() / 256.0 );
  msg.mutable_background()->set_a( 1.0 );
  this->scenePub->Publish(msg);
}

void EditSceneWidget::FogColor()
{
  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  std::ostringstream style;
  style << "background-color:rgb(" << color.red() << "," << color.green() << "," << color.blue() << ");";
  this->fogColorButton->setAutoFillBackground( true );
  this->fogColorButton->setStyleSheet(style.str().c_str());

  /*msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_background()->set_r( color.red() / 256.0 );
  msg.mutable_background()->set_g( color.green() / 256.0 );
  msg.mutable_background()->set_b( color.blue() / 256.0 );
  msg.mutable_background()->set_a( 1.0 );
  this->scenePub->Publish(msg);
  */
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
