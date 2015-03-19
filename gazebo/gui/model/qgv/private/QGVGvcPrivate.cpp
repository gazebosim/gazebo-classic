#include "QGVGvcPrivate.h"

QGVGvcPrivate::QGVGvcPrivate(GVC_t *_gvccontext)
{
		setContext(_gvccontext);
}

void QGVGvcPrivate::setContext(GVC_t *_gvccontext)
{
	_context = _gvccontext;
}

GVC_t* QGVGvcPrivate::context() const
{
	return _context;
}
