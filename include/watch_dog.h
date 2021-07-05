//watch_dog.h
#ifndef __WATCH_DOG__H_
#define __WATCH_DOG__H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#ifndef WINNT
#include <linux/watchdog.h>
#endif
#include "usv_include.h"
extern void initWatchDog(void);
extern void feedWatchDog(void);
extern void disableWatchDog(void);

#endif /*__WATCH_DOG__H_*/