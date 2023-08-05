// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StaticLibrary.hpp"

#include <cstdio>
#include <termios.h>

using namespace teo;

/* Get a single char from stdin:
   Capture characters from standard input without waiting for enter to be pressed
*/
int StaticLibrary::getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr(0, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( 0, TCSANOW, &newattr);
    ch = getchar();
    tcsetattr(0, TCSANOW, &oldattr);
    return(ch);
}
