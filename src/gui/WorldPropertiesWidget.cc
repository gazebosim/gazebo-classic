#include <QtGui>

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/WorldPropertiesWidget.hh"

using namespace gazebo;
using namespace gui;

WorldPropertiesWidget::WorldPropertiesWidget( QWidget *parent )
  : QWidget(parent)
{
  this->setWindowTitle("Gazebo: World");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QTabWidget *tabWidget = new QTabWidget;
  this->sceneWidget = new SceneWidget;
  this->physicsWidget = new PhysicsWidget;

  tabWidget->addTab(this->sceneWidget, tr("Scene") );
  tabWidget->addTab(this->physicsWidget, tr("Physics") );

  mainLayout->addWidget(tabWidget);

  this->setLayout(mainLayout);
  this->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );

}

WorldPropertiesWidget::~WorldPropertiesWidget()
{
}

void WorldPropertiesWidget::showEvent(QShowEvent * /*_event*/)
{
  this->sceneWidget->Init();
}

void WorldPropertiesWidget::closeEvent(QCloseEvent * /*_event*/)
{
  this->sceneWidget->initialized = false;
  std::cout << "Close\n";
}

PhysicsWidget::PhysicsWidget(QWidget *parent )
  : QWidget(parent)
{
  this->setWindowTitle("Gazebo: Physics");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->setLayout(mainLayout);
  //this->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->physicsPub = this->node->Advertise<msgs::Scene>("~/physics");
}

PhysicsWidget::~PhysicsWidget()
{
}


SceneWidget::SceneWidget(QWidget *parent )
  : QWidget(parent)
{
  this->initialized = false;

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

  this->fogBox = new QGroupBox(tr("Fog"));
  this->fogBox->setCheckable(true);
  this->fogBox->setStyleSheet(tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));

  QVBoxLayout *fogBoxLayout = new QVBoxLayout;

  QHBoxLayout *fogTypeLayout = new QHBoxLayout;
  this->fogTypeBox = new QComboBox();
  this->fogTypeBox->addItem(tr("Linear"));
  this->fogTypeBox->addItem(tr("Exponential"));
  this->fogTypeBox->addItem(tr("Squared"));
  QLabel *fogTypeLabel = new QLabel(tr("Type:"));
  fogTypeLayout->addWidget( fogTypeLabel );
  fogTypeLayout->insertStretch(1,0);
  fogTypeLayout->addWidget( this->fogTypeBox );

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

  this->fogBox->setLayout(fogBoxLayout);

  mainLayout->addWidget(shadowsBox);
  mainLayout->addWidget(ambientBox);
  mainLayout->addWidget(backgroundBox);
  mainLayout->addWidget(this->fogBox);
  mainLayout->insertStretch(4,0);

  this->setLayout(mainLayout);
  //this->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");
  this->sceneSub = this->node->Subscribe("~/scene", &SceneWidget::ReceiveSceneMsg, this);

  this->sceneRequestPub = this->node->Advertise<msgs::Request>("~/publish_scene");
  connect( this->ambientColorButton, SIGNAL(clicked()), 
           this, SLOT(OnAmbientColor()) );
  connect( this->backgroundColorButton, SIGNAL(clicked()), 
           this, SLOT(OnBackgroundColor()) );
  connect( this->fogColorButton, SIGNAL(clicked()), 
           this, SLOT(OnFogColor()) );

  connect( this->shadowsButton, SIGNAL(toggled(bool)), 
           this, SLOT(OnShadows(bool)) );

  connect( this->fogStart, SIGNAL(editingFinished()), 
           this, SLOT(OnFogStart()) );
  connect( this->fogEnd, SIGNAL(editingFinished()), 
           this, SLOT(OnFogEnd()) );
  connect( this->fogDensitySpin, SIGNAL(valueChanged(double)), 
           this, SLOT(OnFogDensity(double)) );
  connect( this->fogTypeBox, SIGNAL(currentIndexChanged(int)), 
           this, SLOT(OnFogType(int)) );


  connect( this->fogBox, SIGNAL(toggled(bool)), 
           this, SLOT(OnFogToggle(bool)) );
}

SceneWidget::~SceneWidget()
{
  std::cout << "Destructor\n";
}

void SceneWidget::Init()
{
  this->initialized = false;
  std::cout << "Scene Init\n";
  msgs::Request req;
  req.set_request("publish");

  this->sceneRequestPub->Publish(req);
}

void SceneWidget::OnFogToggle(bool _value)
{
  if (!this->initialized)
    return;

  gzdbg << "Send Fog Message\n";
  msgs::Scene msg;
  msgs::Init(msg, "default");
  if (_value)
  {
    int index = this->fogTypeBox->currentIndex();
    this->OnFogType(index);
  }
  else
    msg.mutable_fog()->set_type( msgs::Fog::NONE );

  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogType(int _index)
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  msgs::Init(msg, "default");

  gzdbg << "OnFogType[" << _index << "]\n";

  if (_index == 0)
    msg.mutable_fog()->set_type( msgs::Fog::LINEAR );
  else if (_index == 1)
    msg.mutable_fog()->set_type( msgs::Fog::EXPONENTIAL );
  else 
    msg.mutable_fog()->set_type( msgs::Fog::EXPONENTIAL2 );
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogDensity(double _density)
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_fog()->set_density(_density );
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogStart()
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  msgs::Init(msg, "default");
  std::string value = this->fogStart->text().toStdString();
  if (!value.empty())
  {
    msg.mutable_fog()->set_start(boost::lexical_cast<double>( value ));
    this->scenePub->Publish(msg);
  }
}

void SceneWidget::OnFogEnd()
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  msgs::Init(msg, "default");
  std::string value = this->fogEnd->text().toStdString();
  if (!value.empty())
  {
    msg.mutable_fog()->set_end(boost::lexical_cast<double>( value ));
    this->scenePub->Publish(msg);
  }
}

void SceneWidget::OnAmbientColor()
{

  if (!this->initialized)
    return;

  gzdbg << "OnAmbientColor\n";
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

void SceneWidget::OnBackgroundColor()
{
  if (!this->initialized)
    return;
  gzdbg << "OnBackgroundColor\n";

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

void SceneWidget::OnFogColor()
{
  if (!this->initialized)
    return;


  gzdbg << "OnFogColor\n";
  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  std::ostringstream style;
  style << "background-color:rgb(" << color.red() << "," << color.green() << "," << color.blue() << ");";
  this->fogColorButton->setAutoFillBackground( true );
  this->fogColorButton->setStyleSheet(style.str().c_str());

  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.mutable_fog()->mutable_color()->set_r( color.red() / 256.0 );
  msg.mutable_fog()->mutable_color()->set_g( color.green() / 256.0 );
  msg.mutable_fog()->mutable_color()->set_b( color.blue() / 256.0 );
  msg.mutable_fog()->mutable_color()->set_a( 1.0 );
  this->scenePub->Publish(msg);
}

void SceneWidget::OnShadows(bool _state)
{
  if (!this->initialized)
    return;

  gzdbg << "OnShadows\n";
  msgs::Scene msg;
  msgs::Init(msg, "default");
  msg.set_shadows( _state );
  this->scenePub->Publish(msg);
}

void SceneWidget::ReceiveSceneMsg(const boost::shared_ptr<msgs::Scene const> &_msg)
{
  if (this->initialized)
    return;

  if (_msg->has_ambient())
  {
    std::ostringstream style;
    style << "background-color:rgb(" << _msg->ambient().r()*256 
      << "," << _msg->ambient().g() * 256 << "," 
      << _msg->ambient().b()*256 << ");";
    this->ambientColorButton->setAutoFillBackground( true );
    this->ambientColorButton->setStyleSheet(style.str().c_str());
  }

  if (_msg->has_background())
  {
    std::ostringstream style;
    style << "background-color:rgb(" << _msg->background().r()*256 
      << "," << _msg->background().g() * 256 << "," 
      << _msg->background().b()*256 << ");";
    this->backgroundColorButton->setAutoFillBackground( true );
    this->backgroundColorButton->setStyleSheet(style.str().c_str());
  }

  if (_msg->has_shadows() && _msg->shadows())
    this->shadowsButton->setCheckState(Qt::Checked);
  else
    this->shadowsButton->setCheckState(Qt::Unchecked);

  if (_msg->has_fog() && _msg->fog().type() != msgs::Fog::NONE)
  {
    this->fogBox->setChecked(true);

    if (_msg->fog().has_color())
    {
      std::ostringstream style;
      style << "background-color:rgb(" 
        << _msg->fog().color().r()*256 
        << "," << _msg->fog().color().g() * 256 << "," 
        << _msg->fog().color().b()*256 << ");";
      this->fogColorButton->setAutoFillBackground( true );
      this->fogColorButton->setStyleSheet(style.str().c_str());
    }

    if (_msg->fog().has_density())
      this->fogDensitySpin->setValue( _msg->fog().density() );

     if (_msg->fog().has_start())
      this->fogStart->setText( tr( boost::lexical_cast<std::string>(_msg->fog().start()).c_str())  );

      if (_msg->fog().has_end())
      this->fogEnd->setText( tr( boost::lexical_cast<std::string>(_msg->fog().end()).c_str())  );
   
  }
  else
  {
    this->fogBox->setChecked(false);
  }

  this->initialized = true;
}
