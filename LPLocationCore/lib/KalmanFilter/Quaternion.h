#pragma once

#include "esp_dsp.h"
#include <stdio.h>

#include <cmath>

class Vector2 {
public:
    float x, y;

    Vector2(float x = 0, float y = 0) : x(x), y(y) {}

    void print() const {
        printf("(%f, %f)\n", x, y);
    }
};

class Vector3 {
public:
    float x, y, z;

    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    void print() const {
        printf("(%f, %f, %f)\n", x, y, z);
    }
};

class Quaternion {
private:
    float data[4]; // [w, x, y, z]

public:
    Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f)
        : data{w, x, y, z} {}

    void normalize() {
        // Use ESP-DSP for vector operations
        float mag_sq;
        dsps_dotprod_f32(data, data, &mag_sq, 4);
        float inv_mag = 1.0f / sqrtf(mag_sq);
        dsps_mulc_f32(data, data, 4, inv_mag, 1);
    }

    void get_rotation_matrix(float* matrix) const {
        // Convert to rotation matrix using ESP-DSP-friendly layout
        const float qw = data[0], qx = data[1], qy = data[2], qz = data[3];
        
        // First row
        matrix[0] = 1 - 2*qy*qy - 2*qz*qz;
        matrix[1] = 2*qx*qy - 2*qz*qw;
        matrix[2] = 2*qx*qz + 2*qy*qw;
        
        // Second row
        matrix[3] = 2*qx*qy + 2*qz*qw;
        matrix[4] = 1 - 2*qx*qx - 2*qz*qz;
        matrix[5] = 2*qy*qz - 2*qx*qw;
        
        // Third row
        matrix[6] = 2*qx*qz - 2*qy*qw;
        matrix[7] = 2*qy*qz + 2*qx*qw;
        matrix[8] = 1 - 2*qx*qx - 2*qy*qy;
    }

    Vector3 rotate(const Vector3& vec) const {
        float rot_matrix[9];
        get_rotation_matrix(rot_matrix);
        
        float input[3] = {vec.x, vec.y, vec.z};
        float output[3];
        
        // Use ESP-DSP matrix multiplication
        dsps_mat_mult_f32(rot_matrix, input, output, 3, 3, 1);
        return Vector3(output[0], output[1], output[2]);
    }

    Vector2 rotate(const Vector2& vec) const {
        // Convert to 3D vector, rotate, then drop Z component
        Vector3 result = rotate(Vector3(vec.x, vec.y, 0));
        return Vector2(result.x, result.y);
    }
};


int main() {
    // Create a 90-degree rotation about Z-axis
    Quaternion q(0.7071, 0, 0, 0.7071); // cos(45°), 0, 0, sin(45°)
    q.normalize();

    Vector2 local_vec(1, 0);
    Vector3 local_vec3d(1, 0, 0);

    printf("Original 2D vector: ");
    local_vec.print();

    printf("Original 3D vector: ");
    local_vec3d.print();

    Vector2 global_vec = q.rotate(local_vec);
    Vector3 global_vec3d = q.rotate(local_vec3d);

    printf("Rotated 2D vector: ");
    global_vec.print();

    printf("Rotated 3D vector: ");
    global_vec3d.print();

    return 0;
}