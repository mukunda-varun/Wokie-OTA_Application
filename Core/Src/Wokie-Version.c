/*
 * Wokie-Version.c
 *
 *  Created on: Aug 9, 2021
 *      Author: Varun
 */
#include "main.h"

#if	OTA_EN	== 1
__attribute__((__section__(".user_data")))  char version[20] = "MukWok-0.1";
__attribute__((__section__(".user_data")))  int dataval = 'B';
#endif


