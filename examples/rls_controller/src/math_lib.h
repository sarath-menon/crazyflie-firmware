#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

// Define the Matrix structure
typedef struct {
    double* data;
    int rows;
    int cols;
} Matrix;

// Function prototypes for matrix operations
void matMul(Matrix* result, Matrix* a, Matrix* b);
void matSub(Matrix* result, Matrix* a, Matrix* b);
void matCopy(Matrix* dest, Matrix* src);
void getSubMatrix(Matrix* result, Matrix* src, int* indices, int num_indices);

#endif // MATRIX_OPERATIONS_H