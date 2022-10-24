#pragma once

#include <QtCore/qglobal.h>
#include<QObject>

#ifndef BUILD_STATIC
# if defined(MOTORMOVE_LIB)
#  define MOTORMOVE_EXPORT Q_DECL_EXPORT
# else
#  define MOTORMOVE_EXPORT Q_DECL_IMPORT
# endif
#else
# define MOTORMOVE_EXPORT
#endif
