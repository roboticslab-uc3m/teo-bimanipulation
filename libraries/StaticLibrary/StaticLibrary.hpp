// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <termios.h>

#ifndef __STATIC_LIBRARY_HPP__
#define __STATIC_LIBRARY_HPP__

namespace teo{

/**
 * @ingroup staticLibrary
 *
 * @brief staticLibrary
 *
 */
class StaticLibrary {
    public:
        static int getch();
};


#endif  // __STATIC_LIBRARY_HPP__

}
