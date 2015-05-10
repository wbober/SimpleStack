#ifndef _LIST_H_
#define _LIST_H_

#include "defines.h"

typedef struct _list_item_t {
    int8_t i_next;
    ptr_t i_data;        
} list_item_t, *plist_item_t;

typedef struct _list_t {
    uint8_t l_count;
    uint8_t l_size;
    int8_t l_head;
    int8_t l_tail;
    list_item_t *l_items; 
} list_t, *plist_t;

#define LIST_INIT(_SIZE) (&((list_t){0, _SIZE, 0, 0, ( list_item_t[_SIZE]){}}))

plist_t list_create(size_t);
uint8_t list_push(plist_t, ptr_t);
ptr_t list_pop(plist_t);
ptr_t list_get(plist_t, uint8_t);
ptr_t list_tail(plist_t);
ptr_t list_head(plist_t);
void list_destroy(plist_t *);

#endif
