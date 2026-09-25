#ifndef KEY_H
#define KEY_H
#include "ti_msp_dl_config.h"

typedef enum { KEY_NONE=0, KEY_OK, KEY_BACK, KEY_UP, KEY_DOWN } key_t;

void key_init(void);
key_t key_read(void);
#endif
