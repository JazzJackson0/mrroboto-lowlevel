#pragma once
#include <math.h>
#include <stdlib.h>

#define DEGREE_TO_RAD (3.141592 / 180)

typedef struct q Quaternion;

//DECLARATIONS
/**
 * @brief Add two Quaternions
 * 
 * @param q Left quaternion
 * @param p Right quaternion
 * @return Quaternion 
 */
Quaternion AddQuaternions(Quaternion q, Quaternion p);
/**
 * @brief Subtract two Quaternions
 * 
 * @param q Left quaternion
 * @param p Right quaternion
 * @return Quaternion 
 */
Quaternion SubtractQuaternions(Quaternion q, Quaternion p);
/**
 * @brief Multiply Quaternion by a scalar
 * 
 * @param q Quaternion
 * @param scalar Scalar
 */
void ScalarMultiplyQuaternion(Quaternion *q, float scalar);
/**
 * @brief Multiply two quaternions
 * 
 * @param q Left quaternion
 * @param p Right quaternion
 * @return ** Quaternion Quaternion Product
 */
Quaternion MultiplyQuaternions(Quaternion q, Quaternion p);
/**
 * @brief Divide two quaternions
 * 
 * @param q Left quaternion
 * @param p Right quaternion
 * @return ** Quaternion Quaternion Quotient
 */
Quaternion DivideQuaternions(Quaternion q, Quaternion p);
/**
 * @brief Rotate a given vector around a given quaternion
 * 
 * @param vector Vector t rotate
 * @param q Quaternion to rotate vector around.
 * @return ** Quaternion Resulting quaternion
 */
void Rotate_Vector(float *vector, Quaternion q);
/**
 * @brief Rotate quaternion by given angle
 * 
 * @param q Quaternion to rotate
 * @param angle Angle of rotation
 * @return ** Quaternion Rotated quaternion.
 */
void RotateByAngle(Quaternion *q, float angle);
/**
 * @brief Normalize quaternion
 * 
 * @param q Quaternion to normalize
 */
void NormalizeQuaternion(Quaternion *q);
/**
 * @brief Get length of quaternion
 * 
 * @param q Quaternion to get length of
 * @return ** float Quaternion length
 */
float Get_QuaternionLength(Quaternion q);
/**
 * @brief Get inverse of quaternion
 * 
 * @param q Quaternion to invert
 * @return ** Quaternion Quaternion inverse
 */
Quaternion Get_QuaternionInverse(Quaternion q);
/**
 * @brief Get conjugate of quaternion
 * 
 * @param q Quaternion to congugate
 * @return ** Quaternion Quaternion conjugate
 */
Quaternion Get_QuaternionConjugate(Quaternion q);


//Structs
struct q {
    float q0;
    float q1;
    float q2;
    float q3;
};

