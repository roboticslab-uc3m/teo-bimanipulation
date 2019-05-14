// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <termios.h>

#ifndef __EXAMPLE_LIBRARY_HPP__
#define __EXAMPLE_LIBRARY_HPP__

namespace teo{

/**
 * @ingroup exampleLibrary
 *
 * @brief exampleLibrary
 *
 */
class ExampleLibrary {
    public:
        static int getch();
};


#endif  // __EXAMPLE_LIBRARY_HPP__

}