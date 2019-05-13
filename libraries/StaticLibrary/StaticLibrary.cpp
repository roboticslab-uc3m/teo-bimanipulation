// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StaticLibrary.hpp"


namespace teo{

/* Get a single char from stdin:
   Capture characters from standard input without waiting for enter to be pressed
*/

    int ExampleLibrary::getch(void)
    {
       struct termios oldattr, newattr;
       int ch;
       tcgetattr(0, &oldattr);
       newattr=oldattr;
       newattr.c_lflag &= ~( ICANON | ECHO );
       tcsetattr( 0, TCSANOW, &newattr);
       ch=getchar();
       tcsetattr(0, TCSANOW, &oldattr);
       return(ch);
    }

} // namespace teo
