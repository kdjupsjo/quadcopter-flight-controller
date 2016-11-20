/*
Mailbox communication from PWMIN
*/

#ifndef __ANALYS_H
#define __ANALYS_H

#include "main.h"


/* Mailboxes */
extern osMailQId analys_mailbox;

void StartAnalysTask(void const * argument);

#endif