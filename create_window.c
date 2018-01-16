#include <stdio.h>
#include <stdlib.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

void create_window()
{
    Display* xdisp = XOpenDisplay( NULL );

    if( xdisp == NULL ) {
        printf("!!! XOpenDisplay\n");
        exit(1);
    }

    Window rootWin = DefaultRootWindow( xdisp );

    int screen = DefaultScreen( xdisp );
    unsigned long white = WhitePixel( xdisp, screen );
    unsigned long black = BlackPixel( xdisp, screen );

    Window window = XCreateSimpleWindow( xdisp, rootWin,
                                         100, 100,
                                         800, 600,
                                         2,
                                         black, white );


    XSelectInput( xdisp, window, KeyPressMask );

    XMapWindow( xdisp, window );

    XFlush(xdisp);

}

int main()
{
    create_window();
    for(;;){}

    return 0;
}
