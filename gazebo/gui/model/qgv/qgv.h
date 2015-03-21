#ifndef QGV_H
#define QGV_H

#include <QtGlobal>

#ifdef QGVCORE_LIB
	#define QGVCORE_EXPORT Q_DECL_EXPORT
#else
	#define QGVCORE_EXPORT Q_DECL_IMPORT
#endif

#endif // QGV_H
