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
  this->modelTreeWidget->setContextMenuPolicy( Qt::CustomContextMenu );
  this->modelTreeWidget->header()->hide();
  connect(this->modelTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)) );
  connect(this->modelTreeWidget, 
      SIGNAL( customContextMenuRequested(const QPoint &)),
      this, SLOT(OnCustomContextMenu(const QPoint &)));

  mainLayout->addWidget(this->modelTreeWidget);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(2,2,2,2);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->entitiesRequestPub = this->node->Advertise<msgs::Request>("~/request_entities");
  this->entitiesSub = this->node->Subscribe("~/entities", &ModelListWidget::OnEntities, this);
  this->entityPub = this->node->Advertise<msgs::Entity>("~/entity");
  this->newEntitySub = this->node->Subscribe("~/entity", &ModelListWidget::OnEntity, this);

  msgs::Request msg;
  msg.set_index( 1 );
  msg.set_request( "entities" );

  this->entitiesRequestPub->Publish( msg );

  this->moveToAction = new QAction(tr("Move To"), this);
  this->moveToAction->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAction, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->deleteAction = new QAction(tr("Delete"), this);
  this->deleteAction->setStatusTip(tr("Delete the selection"));
  connect(this->deleteAction, SIGNAL(triggered()), this, SLOT(OnDelete()));
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

void ModelListWidget::OnEntity( const boost::shared_ptr<msgs::Entity const> &_msg )
{
  std::string name = _msg->name();

  QList<QTreeWidgetItem*> list = this->modelTreeWidget->findItems( 
      name.c_str(), Qt::MatchExactly);

  if (list.size() == 0)
  {
    if (!_msg->has_request_delete() || !_msg->request_delete())
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
          QStringList(QString("%1").arg( QString::fromStdString(name)) ));
      this->modelTreeWidget->addTopLevelItem(topItem);
    }
  }
  else
  {
    if (_msg->has_request_delete() && _msg->request_delete())
    {
      QList<QTreeWidgetItem*>::Iterator iter;
      for (iter = list.begin(); iter != list.end(); iter++)
      {
        int i = this->modelTreeWidget->indexOfTopLevelItem(*iter);
        this->modelTreeWidget->takeTopLevelItem(i);
        delete *iter;
      }
    }
  }

}

void ModelListWidget::OnEntities( const boost::shared_ptr<msgs::Entities const> &_msg )
{
  for (int i=0; i < _msg->entities_size(); i++)
  {
    std::string name = _msg->entities(i).name();

    // Create a top-level tree item for the path
    QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
        QStringList(QString("%1").arg( QString::fromStdString(name)) ));
    this->modelTreeWidget->addTopLevelItem(topItem);
  }
}

void ModelListWidget::OnDelete()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  msgs::Entity msg;
  msgs::Init(msg, modelName);
  msg.set_name( modelName );
  msg.set_request_delete( true );

  this->entityPub->Publish(msg);
}

void ModelListWidget::OnMoveTo()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveTo(modelName);
}

void ModelListWidget::OnCustomContextMenu(const QPoint &_pt)
{
  QTreeWidgetItem *item = this->modelTreeWidget->itemAt(_pt);

  if (item)
  {
    QMenu menu(this->modelTreeWidget);
    menu.addAction(moveToAction);
    menu.addAction(deleteAction);
    menu.exec(this->modelTreeWidget->mapToGlobal(_pt));
  }
}
