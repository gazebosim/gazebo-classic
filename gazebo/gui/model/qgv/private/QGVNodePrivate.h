#ifndef QGVNODEPRIVATE_H
#define QGVNODEPRIVATE_H

#include <cgraph.h>

class QGVNodePrivate
{
	public:
		QGVNodePrivate(Agnode_t *node = NULL);

		void setNode(Agnode_t *node);
		Agnode_t* node() const;

	private:
		Agnode_t* _node;
};

#endif // QGVNODEPRIVATE_H
