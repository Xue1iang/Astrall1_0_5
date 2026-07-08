#ifndef __INPUT_KEYBOARD_H__
#define __INPUT_KEYBOARD_H__

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <functional>

class KeyBoard
{
    public:
        KeyBoard();
        ~KeyBoard();
        void registerEventCb(std::function<void(char key)> cb) { this->cb = cb; }

    private:
        static void *runKeyBoard(void *arg);
        void *run(void *arg);
        std::function<void(char key)> cb;

        pthread_t _tid;
        struct termios _oldSettings, _newSettings;
        fd_set set;
        int res;
        int ret;
        char _c;
};

#endif /* __INPUT_KEYBOARD_H__ */

