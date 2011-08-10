#include <QtGui>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"
#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/ModelListWidget.hh"

using namespace gazebo;
using namespace gui;

ModelListWidget::ModelListWidget( QWidget *parent )
  : QWidget( parent )
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->modelTreeWidget = new QTreeWidget();
  this->modelTreeWidget->setColumnCount(1);
  this->modelTreeWidget->headerItem()->setText(0, tr("Models") );
  connect(this->modelTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)) );

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  this->moveToButton = new QPushButton(tr("MoveTo"));
  connect(this->moveToButton, SIGNAL(clicked()), this, SLOT(OnMoveTo()));

  buttonLayout->addWidget(this->moveToButton);

  mainLayout->addWidget(this->modelTreeWidget);
  mainLayout->addLayout(buttonLayout);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(2,2,2,2);

  QList<QTreeWidgetItem*> items;
  std::list<std::string> gazeboPaths = common::SystemPaths::GetGazeboPaths();

  // Iterate over all the gazebo paths
  for (std::list<std::string>::iterator iter = gazeboPaths.begin(); 
       iter != gazeboPaths.end(); iter++)
  {
    // This is the full model path
    std::string path = (*iter) + common::SystemPaths::GetModelPathExtension();

    // Create a top-level tree item for the path
    QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, QStringList(QString("%1").arg( QString::fromStdString(path)) ));
    this->modelTreeWidget->addTopLevelItem(topItem);

    boost::filesystem::path dir(path);
    boost::filesystem::directory_iterator endIter;
    std::list<boost::filesystem::path> resultSet;

    if ( boost::filesystem::exists(dir) && 
         boost::filesystem::is_directory(dir) )
    {
      // Iterate over all the model in the current gazebo path
      for( boost::filesystem::directory_iterator dirIter(dir); 
           dirIter != endIter; ++dirIter)
      {
        if ( boost::filesystem::is_regular_file(dirIter->status()) )
        {
          // Add a child item for the model
          QTreeWidgetItem *childItem = new QTreeWidgetItem( topItem, 
              QStringList(QString("%1").arg( 
                  QString::fromStdString( dirIter->path().leaf() )) ));
          this->modelTreeWidget->addTopLevelItem(childItem);
        }
      }
    }
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->entitiesRequestPub = this->node->Advertise<msgs::Request>("~/request_entities");
  this->entitiesSub = this->node->Subscribe("~/entities", &ModelListWidget::OnEntities, this);
}

ModelListWidget::~ModelListWidget()
{
}

void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
  }
}

void ModelListWidget::OnEntities( const boost::shared_ptr<msgs::Entities const> &_msg )
{
}

void ModelListWidget::OnMoveTo()
{
}
