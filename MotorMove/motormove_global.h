#pragma once

#include <QtCore/qglobal.h>
#include <QObject>

#ifdef BUILD_STATIC
#define MOTORMOVE_API Q_DECL_EXPORT
#else
#define MOTORMOVE_API Q_DECL_IMPORT
#endif 

/*
#ifndef BUILD_STATIC
# if defined(MOTORMOVE_LIB)
#  define MOTORMOVE_EXPORT Q_DECL_EXPORT
# else
#  define MOTORMOVE_EXPORT Q_DECL_IMPORT
# endif
#else
# define MOTORMOVE_EXPORT
#endif

*/