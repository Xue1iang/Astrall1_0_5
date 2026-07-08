#include "keyboard.h"
#include <iostream>

KeyBoard::KeyBoard()
{
    tcgetattr(fileno(stdin), &_oldSettings);
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno(stdin), TCSANOW, &_newSettings);

    pthread_create(&_tid, NULL, runKeyBoard, (void *)this);
}

KeyBoard::~KeyBoard()
{
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr(fileno(stdin), TCSANOW, &_oldSettings);
}

void *KeyBoard::runKeyBoard(void *arg)
{
    ((KeyBoard *)arg)->run(NULL);
    return NULL;
}

void *KeyBoard::run(void *arg)
{
    while (1)
    {
        FD_ZERO(&set);
        FD_SET(fileno(stdin), &set);

        res = select(fileno(stdin) + 1, &set, NULL, NULL, NULL);
        if (res > 0)
        {
            ret = read(fileno(stdin), &_c, 1);
            if (cb)
	        cb(_c);
	    else
		printf("Keyboard no register callback\r\n");
	    _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}

