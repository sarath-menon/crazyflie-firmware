#include <stdio.h>
#include <stdlib.h>
#include "math_lib.h"

// Matrix multiplication
void matMul(Matrix* result, Matrix* a, Matrix* b) {
    if (a->cols != b->rows) {
        printf("Error: Incompatible dimensions for matrix multiplication\n");
        return;
    }
    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < b->cols; j++) {
            double sum = 0.0;
            for (int k = 0; k < a->cols; k++) {
                sum += a->data[i * a->cols + k] * b->data[k * b->cols + j];
            }
            result->data[i * result->cols + j] = sum;
        }
    }
}

// Matrix subtraction
void matSub(Matrix* result, Matrix* a, Matrix* b) {
    if (a->rows != b->rows || a->cols != b->cols) {
        printf("Error: Incompatible dimensions for matrix subtraction\n");
        return;
    }
    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < a->cols; j++) {
            result->data[i * a->cols + j] = a->data[i * a->cols + j] - b->data[i * b->cols + j];
        }
    }
}

// Matrix copy
void matCopy(Matrix* dest, Matrix* src) {
    if (dest->rows != src->rows || dest->cols != src->cols) {
        printf("Error: Incompatible dimensions for matrix copy\n");
        return;
    }
    for (int i = 0; i < src->rows * src->cols; i++) {
        dest->data[i] = src->data[i];
    }
}

// Extract submatrix based on a list of row indices
void getSubMatrix(Matrix* result, Matrix* src, int* indices, int num_indices) {
    if (result->rows != num_indices || result->cols != src->cols) {
        printf("Error: Incompatible dimensions for extracting submatrix\n");
        return;
    }
    for (int i = 0; i < num_indices; i++) {
        int idx = indices[i];
        if (idx >= src->rows) {
            printf("Error: Index out of bounds for submatrix extraction\n");
            return;
        }
        for (int j = 0; j < src->cols; j++) {
            result->data[i * result->cols + j] = src->data[idx * src->cols + j];
        }
    }
}