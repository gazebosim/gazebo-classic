/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <mutex>
#include <tinyxml2.h>

#include "gazebo/common/Console.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/ExportDialog.hh"
#include "gazebo/gui/plot/Palette.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotManager.hh"
#include "gazebo/gui/plot/PlotWindow.hh"

using namespace gazebo;
using namespace gui;

using namespace tinyxml2;
#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult)	if (a_eResult != XML_SUCCESS) { \
										printf("Error: %i\n", a_eResult); \
										return a_eResult; \
									}
#endif

namespace gazebo {
namespace gui {
/// \brief Private data for the PlotWindow class
class PlotWindowPrivate {
	/// \brief Splitter to hold all the canvases.
public:
	QSplitter *canvasSplitter;

	/// \brief Mutex to protect the canvas updates
public:
	std::mutex mutex;

	/// \brief Flag to indicate whether the plots should be restarted.
public:
	bool restart = false;
};
}
}

// A special list widget that allows dragging of items from it to a
// plot
class DragableListWidget: public QListWidget {
public:
	explicit DragableListWidget(QWidget *_parent) :
			QListWidget(_parent) {
	}

protected:
	virtual void startDrag(Qt::DropActions /*_supportedActions*/) {
		QListWidgetItem *currItem = this->currentItem();
		QMimeData *currMimeData = new QMimeData;
		QByteArray ba;
		ba = currItem->text().toLatin1().data();
		currMimeData->setData("application/x-item", ba);
		QDrag *drag = new QDrag(this);
		drag->setMimeData(currMimeData);
		drag->exec(Qt::LinkAction);
	}

protected:
	virtual Qt::DropActions supportedDropActions() const {
		return Qt::LinkAction;
	}
};

/////////////////////////////////////////////////
PlotWindow::PlotWindow(QWidget *_parent) :
		QWidget(_parent), dataPtr(new PlotWindowPrivate()) {
	this->setWindowIcon(QIcon(":/images/gazebo.svg"));
	this->setWindowTitle("Gazebo: Plotting Utility");
	this->setObjectName("plotWindow");
	this->setWindowFlags(
			Qt::Window | Qt::WindowTitleHint | Qt::WindowCloseButtonHint
					| Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

	// new empty canvas
	this->dataPtr->canvasSplitter = new QSplitter(Qt::Vertical);
	this->AddCanvas();

	// export button
	QPushButton *exportPlotButton = new QPushButton("Export");
	exportPlotButton->setIcon(QIcon(":/images/file_upload.svg"));
	exportPlotButton->setObjectName("plotExport");
	exportPlotButton->setDefault(false);
	exportPlotButton->setAutoDefault(false);
	exportPlotButton->setToolTip("Export plot data");
	QGraphicsDropShadowEffect *exportPlotShadow =
			new QGraphicsDropShadowEffect();
	exportPlotShadow->setBlurRadius(8);
	exportPlotShadow->setOffset(0, 0);
	exportPlotButton->setGraphicsEffect(exportPlotShadow);
	connect(exportPlotButton, SIGNAL(clicked()), this, SLOT(OnExport()));

	// save button
	QPushButton *saveCanvasButton = new QPushButton("Save");
	saveCanvasButton->setObjectName("plotSaveCanvas");
	saveCanvasButton->setDefault(false);
	saveCanvasButton->setAutoDefault(false);
	saveCanvasButton->setToolTip("Save canvas");
	QGraphicsDropShadowEffect *saveCanvasShadow =
			new QGraphicsDropShadowEffect();
	saveCanvasShadow->setBlurRadius(8);
	saveCanvasShadow->setOffset(0, 0);
	saveCanvasButton->setGraphicsEffect(saveCanvasShadow);
	connect(saveCanvasButton, SIGNAL(clicked()), this, SLOT(OnSaveCanvas()));

	// load button
	QPushButton *loadCanvasButton = new QPushButton("Load");
	loadCanvasButton->setObjectName("plotLoadCanvas");
	loadCanvasButton->setDefault(false);
	loadCanvasButton->setAutoDefault(false);
	loadCanvasButton->setToolTip("Load canvas");
	QGraphicsDropShadowEffect *loadCanvasShadow =
			new QGraphicsDropShadowEffect();
	loadCanvasShadow->setBlurRadius(8);
	loadCanvasShadow->setOffset(0, 0);
	loadCanvasButton->setGraphicsEffect(loadCanvasShadow);
	connect(loadCanvasButton, SIGNAL(clicked()), this, SLOT(OnLoadCanvas()));

	// add button
	QPushButton *addCanvasButton = new QPushButton("+");
	addCanvasButton->setObjectName("plotAddCanvas");
	addCanvasButton->setDefault(false);
	addCanvasButton->setAutoDefault(false);
	addCanvasButton->setToolTip("Add a new canvas");
	QGraphicsDropShadowEffect *addCanvasShadow =
			new QGraphicsDropShadowEffect();
	addCanvasShadow->setBlurRadius(8);
	addCanvasShadow->setOffset(0, 0);
	addCanvasButton->setGraphicsEffect(addCanvasShadow);
	connect(addCanvasButton, SIGNAL(clicked()), this, SLOT(OnAddCanvas()));

	QHBoxLayout *addButtonLayout = new QHBoxLayout;
	addButtonLayout->addWidget(exportPlotButton);
	addButtonLayout->addStretch();
	addButtonLayout->addWidget(saveCanvasButton);
	addButtonLayout->addWidget(loadCanvasButton);
	addButtonLayout->addWidget(addCanvasButton);
	addButtonLayout->setAlignment(Qt::AlignRight | Qt::AlignBottom);
	addButtonLayout->setContentsMargins(0, 0, 0, 0);
	addCanvasButton->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

	// Plot layout
	QVBoxLayout *plotLayout = new QVBoxLayout;
	plotLayout->addWidget(this->dataPtr->canvasSplitter);
	plotLayout->addLayout(addButtonLayout);
	plotLayout->setStretchFactor(this->dataPtr->canvasSplitter, 1);
	plotLayout->setStretchFactor(addButtonLayout, 0);

	auto plotFrame = new QFrame;
	plotFrame->setLayout(plotLayout);

	// Palette
	auto plotPalette = new Palette(this);

	auto splitter = new QSplitter(Qt::Horizontal, this);
	splitter->addWidget(plotPalette);
	splitter->addWidget(plotFrame);
	splitter->setCollapsible(0, true);
	splitter->setCollapsible(1, false);

	QList<int> sizes;
	sizes << 30 << 70;
	splitter->setSizes(sizes);

	auto mainLayout = new QHBoxLayout;
	mainLayout->addWidget(splitter);
	mainLayout->setContentsMargins(0, 0, 0, 0);

	this->setLayout(mainLayout);

	QShortcut *space = new QShortcut(Qt::Key_Space, this);
	QObject::connect(space, SIGNAL(activated()), this, SLOT(TogglePause()));

	QTimer *displayTimer = new QTimer(this);
	connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
	displayTimer->start(30);

	PlotManager::Instance()->AddWindow(this);

	this->setMinimumSize(640, 480);
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow() {
	PlotManager::Instance()->RemoveWindow(this);
	this->Clear();
}

/////////////////////////////////////////////////
PlotCanvas* PlotWindow::AddCanvas() {
	PlotCanvas *canvas = new PlotCanvas(this);
	connect(canvas, SIGNAL(CanvasDeleted()), this, SLOT(OnRemoveCanvas()));

	this->dataPtr->canvasSplitter->addWidget(canvas);

	this->UpdateCanvas();

	return canvas;
}

/////////////////////////////////////////////////
void PlotWindow::SavePlotLayout() {
	QString _fileName = "";

	XMLError _XMLError;
	XMLDocument _plotLayout_XMLDocument;
	XMLNode *_rootNode = nullptr;
	XMLElement *_canvas_XMLElement = nullptr;
	XMLElement *_plot_XMLElement = nullptr;
	XMLElement *_variable_XMLElement = nullptr;

	PlotCanvas *_canvas = nullptr;

	_fileName = QFileDialog::getSaveFileName(this, tr("Save Plot Layout"), "~",
			tr("XML File (*.xml)"));

	_rootNode = _plotLayout_XMLDocument.NewElement("PlotLayout");
	_plotLayout_XMLDocument.InsertFirstChild(_rootNode);

	for (int _canvasIndex = 0;
			_canvasIndex < this->dataPtr->canvasSplitter->count();
			++_canvasIndex) {
		_canvas = qobject_cast<PlotCanvas*>(
				this->dataPtr->canvasSplitter->widget(_canvasIndex));
		if (!_canvas) {
			continue;
		}

		_canvas_XMLElement = _plotLayout_XMLDocument.NewElement("Canvas");
		_rootNode->InsertEndChild(_canvas_XMLElement);

		for (const auto &_plot : _canvas->Plots()) {
			_plot_XMLElement = _plotLayout_XMLDocument.NewElement("Plot");
			_canvas_XMLElement->InsertEndChild(_plot_XMLElement);

			for (const auto &_curve : _plot->Curves()) {
				auto _tempCurve = _curve.lock();
				if (!_tempCurve)
					continue;

				std::string _label = _tempCurve->Label();

				_variable_XMLElement = _plotLayout_XMLDocument.NewElement(
						"Variable");

				_variable_XMLElement->SetAttribute("Label", _label.c_str());

				_plot_XMLElement->InsertEndChild(_variable_XMLElement);
			} // for (const auto &_curve : _plot->Curves()) {}
		} // for (const auto &_plot : _canvas->Plots()) {
	} // for (int _canvasIndex = 0; ...

	  // todo: treat XMLError
	_XMLError = _plotLayout_XMLDocument.SaveFile(_fileName.toLocal8Bit());
} // void PlotWindow::SavePlotLayout() {

/////////////////////////////////////////////////
void PlotWindow::LoadPlotLayout() {
	QString _fileName = "";

	XMLError _XMLError;
	XMLDocument _plotLayout_XMLDocument;
	XMLNode *_rootNode = nullptr;
	XMLElement *_canvas_XMLElement = nullptr;
	XMLElement *_plot_XMLElement = nullptr;
	XMLElement *_variable_XMLElement = nullptr;

	PlotCanvas *_canvas = nullptr;

	std::string _label;

	std::vector<IncrementalPlot*> _plotVector;
	IncrementalPlot *_plot = nullptr;
	int _size = 0;

	_fileName = QFileDialog::getOpenFileName(this, tr("Load Plot Layout"), "~",
			tr("XML File (*.xml)"));

	if (_fileName == "") {
		return;
	}

	this->Clear();

	_XMLError = _plotLayout_XMLDocument.LoadFile(_fileName.toLocal8Bit());

	_rootNode = _plotLayout_XMLDocument.FirstChild();
	if (_rootNode == nullptr)
		return;

	_canvas_XMLElement = _rootNode->FirstChildElement("Canvas");
	while (_canvas_XMLElement != nullptr) {
		_canvas = this->AddCanvas();

		if (!_canvas) {
			return;
		}

		_plot_XMLElement = _canvas_XMLElement->FirstChildElement("Plot");
		while (_plot_XMLElement != nullptr) {
			_variable_XMLElement = _plot_XMLElement->FirstChildElement(
					"Variable");
			while (_variable_XMLElement != nullptr) {

				const char *tempChars = _variable_XMLElement->Attribute(
						"Label");
				if (tempChars != nullptr) {
					std::string _label = tempChars;

					_plotVector = _canvas->Plots();
					_size = _plotVector.size();

					if (_size == 0) {
						_canvas->AddVariable(_label);
					} else { // if (_size == 0) {
						_plot = _plotVector.back();
						_canvas->AddVariable(_label, _plot);
					} // } else { // if (_size == 0) {
				} // if (tempChars != nullptr) {

				_variable_XMLElement = _variable_XMLElement->NextSiblingElement(
						"Variable");
			} // while (_variable_XMLElement != nullptr) {

			_plot_XMLElement = _plot_XMLElement->NextSiblingElement("Plot");
		} // while (_plot_XMLElement != nullptr)

		_canvas_XMLElement = _canvas_XMLElement->NextSiblingElement("Canvas");
	} // while (_canvas_XMLElement != nullptr)
} // void PlotWindow::LoadPlotLayout() {

/////////////////////////////////////////////////
void PlotWindow::RemoveCanvas(PlotCanvas *_canvas) {
	int idx = this->dataPtr->canvasSplitter->indexOf(_canvas);
	if (idx < 0)
		return;

	_canvas->hide();
	_canvas->setParent(nullptr);
	_canvas->deleteLater();
}

/////////////////////////////////////////////////
void PlotWindow::Clear() {
	while (this->CanvasCount() > 0u) {
		PlotCanvas *canvas = qobject_cast<PlotCanvas*>(
				this->dataPtr->canvasSplitter->widget(0));
		this->RemoveCanvas(canvas);
	}
}

/////////////////////////////////////////////////
unsigned int PlotWindow::CanvasCount() const {
	return static_cast<unsigned int>(this->dataPtr->canvasSplitter->count());
}

/////////////////////////////////////////////////
void PlotWindow::OnAddCanvas() {
	this->AddCanvas();
}

/////////////////////////////////////////////////
void PlotWindow::OnSaveCanvas() {
	this->SavePlotLayout();
}

/////////////////////////////////////////////////
void PlotWindow::OnLoadCanvas() {
	this->LoadPlotLayout();
}

/////////////////////////////////////////////////
void PlotWindow::OnRemoveCanvas() {
	PlotCanvas *canvas = qobject_cast<PlotCanvas*>(QObject::sender());
	if (!canvas)
		return;

	this->RemoveCanvas(canvas);

	// add an empty canvas if the plot window is now empty
	if (this->dataPtr->canvasSplitter->count() == 0)
		this->AddCanvas();
	else {
		this->UpdateCanvas();
	}
}

/////////////////////////////////////////////////
void PlotWindow::UpdateCanvas() {
	// disable Delete Canvas option in settings if there is only one
	// canvas in the window
	PlotCanvas *plotCanvas = qobject_cast<PlotCanvas*>(
			this->dataPtr->canvasSplitter->widget(0));
	if (plotCanvas) {
		plotCanvas->SetDeleteCanvasEnabled(
				this->dataPtr->canvasSplitter->count() != 1);
	}
}

/////////////////////////////////////////////////
void PlotWindow::Update() {
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
	if (this->dataPtr->restart) {
		for (int i = 0; i < this->dataPtr->canvasSplitter->count(); ++i) {
			PlotCanvas *canvas = qobject_cast<PlotCanvas*>(
					this->dataPtr->canvasSplitter->widget(i));
			if (!canvas)
				continue;
			canvas->Restart();
		}
		this->dataPtr->restart = false;
	}

	for (int i = 0; i < this->dataPtr->canvasSplitter->count(); ++i) {
		PlotCanvas *canvas = qobject_cast<PlotCanvas*>(
				this->dataPtr->canvasSplitter->widget(i));
		if (!canvas)
			continue;
		canvas->Update();
	}
}

/////////////////////////////////////////////////
void PlotWindow::Restart() {
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
	this->dataPtr->restart = true;
}

/////////////////////////////////////////////////
void PlotWindow::TogglePause() {
	MainWindow *mainWindow = gui::get_main_window();
	if (!mainWindow)
		return;

	if (mainWindow->IsPaused())
		mainWindow->Play();
	else
		mainWindow->Pause();
}

/////////////////////////////////////////////////
void PlotWindow::OnExport() {
	// Get the plots that have data.
	std::list<PlotCanvas*> plots;
	for (int i = 0; i < this->dataPtr->canvasSplitter->count(); ++i) {
		bool hasData = false;
		PlotCanvas *canvas = qobject_cast<PlotCanvas*>(
				this->dataPtr->canvasSplitter->widget(i));

		if (!canvas)
			continue;

		for (const auto &plot : canvas->Plots()) {
			for (const auto &curve : plot->Curves()) {
				auto c = curve.lock();
				if (!c)
					continue;

				hasData = hasData || c->Size() > 0;
			}
		}

		if (hasData)
			plots.push_back(canvas);
	}

	// Display an error message if no plots have data.
	if (plots.empty()) {
		QMessageBox msgBox(QMessageBox::Information,
				QString("Unable to export"),
				QString(
						"No data to export.\nAdd variables with data to a graph first."),
				QMessageBox::Close, this,
				Qt::Window | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint
						| Qt::CustomizeWindowHint);
		msgBox.exec();
	} else {
		ExportDialog *dialog = new ExportDialog(this, plots);
		dialog->setModal(true);
		dialog->show();
	}
}

/////////////////////////////////////////////////
std::list<PlotCanvas*> PlotWindow::Plots() {
	std::list<PlotCanvas*> plots;

	for (int i = 0; i < this->dataPtr->canvasSplitter->count(); ++i) {
		PlotCanvas *canvas = qobject_cast<PlotCanvas*>(
				this->dataPtr->canvasSplitter->widget(i));

		if (!canvas)
			continue;
		plots.push_back(canvas);
	}

	return plots;
}
