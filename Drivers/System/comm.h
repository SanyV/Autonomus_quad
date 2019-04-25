
#pragma once
#ifndef _comm_h
#define _comm_h


#define COMM_STACK_SIZE		320   // must be evenly divisible by 8
#define COMM_PRIORITY		40
#define COMM_NOTICE_DEPTH	6    // очередь уведомлений
#define COMM_NOTICE_LENGTH	64    // размер сообщения

int CommInit (void);

#endif
