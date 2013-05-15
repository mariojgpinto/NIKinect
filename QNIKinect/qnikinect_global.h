#ifndef QNIKINECT_GLOBAL_H
#define QNIKINECT_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef QNIKINECT_LIB
# define QNIKINECT_EXPORT Q_DECL_EXPORT
#else
# define QNIKINECT_EXPORT Q_DECL_IMPORT
#endif

#endif // QNIKINECT_GLOBAL_H
