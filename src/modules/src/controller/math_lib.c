#include <stdio.h>
#include <stdlib.h>
#include "math_lib.h"

// create an identity matrix of given size
void create_identity_matrix(float *matrix, int size)
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            matrix[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}