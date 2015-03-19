#ifndef QGVGVCPRIVATE_H
#define QGVGVCPRIVATE_H

#include <gvc.h>

class QGVGvcPrivate
{
	public:
		QGVGvcPrivate(GVC_t *context = NULL);

		void setContext(GVC_t *context);
		GVC_t* context() const;

		// operators to implicit cast from QGVGvcPrivate* into GVC_t* seems not to work,
		// because of typedef GVC_t
//		inline operator const GVC_t* () const
//		{
//			return const_cast<VC_t*>(context());
//		}

//		inline operator struct GVC_t* ()
//		{
//			return context();
//		}

	private:
		GVC_t* _context;
};

#endif // QGVGVCPRIVATE_H
