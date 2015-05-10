#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "list.h"

plist_t list_create(size_t s)
{
    plist_t l;
    uint8_t i;
    
    l = (plist_t) malloc(sizeof(list_t));
    l->l_items = (plist_item_t)malloc(s*sizeof(list_item_t));
    l->l_head = l->l_tail = 0;
    l->l_count = 0;
    l->l_size = s;

    for (i = 0; i < l->l_size; i++) {
        l->l_items[i].i_next = -1;
        l->l_items[i].i_data = NULL;
    }

    return l;
}

/**
    Insert an item at the end of the list.
 */
uint8_t list_push(plist_t l, ptr_t p) 
{
    plist_item_t i = l->l_items;    

    // exit if the list is full
    if (l->l_count >= l->l_size)
        return 0;

    l->l_count++;

    if (l->l_count > 1) {
        // look for a free place
        while (i < l->l_items + l->l_size && (i == l->l_items + l->l_tail || i->i_next != -1))
            i++;
        // link tail
        l->l_items[l->l_tail].i_next = i - l->l_items;
        l->l_tail = i - l->l_items;
    } else 
        i = l->l_items + l->l_head;

    // set item in the free place
    i->i_data = p;

    return 1;
}

/**
    Pop the list item from the list head.
 */
ptr_t list_pop(plist_t l)
{
    plist_item_t i;
    ptr_t p;

    if (!l->l_count)
        return NULL;
    
    i = &l->l_items[l->l_head];
    p = i->i_data;

    l->l_count--;

    // link head to the next item
    if (l->l_count)
        l->l_head = i->i_next;

    // clear pop'ed item
    i->i_data = NULL;
    i->i_next = -1;
  
    return p;
}

ptr_t list_get(plist_t l, uint8_t index)
{
    plist_item_t i = &l->l_items[l->l_head];

    if (index >= l->l_count)
        return NULL;

    while (index--)
        i++;

    return i->i_data;
}

/**
    Return list head
 */
inline ptr_t list_head(plist_t l)
{
    return l->l_count ? l->l_items[l->l_head].i_data : NULL;
}

/**
    Return list tail
*/
inline ptr_t list_tail(plist_t l)
{
    return l->l_count ? l->l_items[l->l_tail].i_data : NULL;
}

/**
    Destroy the list
 */
void list_destroy(plist_t *l) 
{
    if (!*l)
        return;

    if ((*l)->l_items)
        free((*l)->l_items);
    free(*l);

    l = NULL;
}
