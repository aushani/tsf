#pragma once
#ifndef _FLOW_H_
#define _FLOW_H_

#include <stdint.h>

typedef struct {

    int32_t u;
    int32_t v;

    //int32_t idx; // To help build, where does this flow belong?
    int32_t valid;

} flow_t;

#endif
