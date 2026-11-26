// centroid_filter.h
#ifndef CENTROID_FILTER_H
#define CENTROID_FILTER_H

#include "centroid.h"

void centroid_filter_init(void);
void centroid_filter_apply(Centroids *c, float dt);

#endif
