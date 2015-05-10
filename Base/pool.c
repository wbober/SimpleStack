#include "defines.h"
#include "pool.h"


void* pool_get(ppool_t pool) {
    uint8_t i = 0, j;
    uint8_t *used = pool->p_used;

    while (i < pool->p_size) {
        // check if there is a free block in the given 8
        if (*used != 0xFF) {
            j = 0;
            while ((*used >> j) & 0x1)
                j++;
            *used |= _BV(j);
            return pool->p_block + i*8 + j*pool->p_data_size;
        } else {
            used += 1;
            i += 8;
        }
    }
    return NULL;
}

void pool_put(ppool_t pool, void* element) {
    // TODO: check that element is in
    uint8_t i = (element - pool->p_block) / pool->p_data_size;
    pool->p_used[i/8] &= ~(_BV(i%8));
    return;
}
