#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

// Define the Matrix structure
typedef struct
{
    double *data;
    int rows;
    int cols;
} Matrix;
void create_identity_matrix(float *matrix, int size);

#endif // MATRIX_OPERATIONS_H
