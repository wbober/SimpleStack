#ifndef POOL_H
#define POOL_H

#include <stdio.h>
#include "defines.h"

typedef struct {
    uint8_t p_size;
    uint8_t p_data_size;
    uint8_t *p_used;
    void* p_block;
} pool_t, *ppool_t;


#define POOL_INIT(_TYPE, _SIZE)\
    (&((pool_t){\
        .p_size = _SIZE,\
        .p_data_size = sizeof(_TYPE),\
        .p_block = (_TYPE[_SIZE]){},\
        .p_used = (uint8_t[_SIZE / sizeof(uint8_t)]){}\
     }))
//

void* pool_get(ppool_t pool);
void pool_put(ppool_t pool, void* element);

#endif
