// centroid.h
#ifndef CENTROID_H
#define CENTROID_H

#include <stdint.h>

typedef struct {
    float position;
    float peak;
    uint8_t valid;
} Centroid;

typedef struct {
    Centroid up;
    Centroid down;
    Centroid left;
    Centroid right;
    Centroid in_up;
    Centroid in_down;
    Centroid in_left;
    Centroid in_right;
} Centroids;

void centroids_find(uint16_t *raw[8], const uint16_t *edge_len, Centroids *c);

#endif