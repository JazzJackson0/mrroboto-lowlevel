#include "../include/Quaternions.h"


Quaternion AddQuaternions(Quaternion q, Quaternion p) {
   
    Quaternion r = { 
        q.q0 + p.q0,
        q.q1 + p.q1,
        q.q2 + p.q2,
        q.q3 + p.q3
    };

    return r;
}

Quaternion SubtractQuaternions(Quaternion q, Quaternion p) {
    Quaternion r = { 
        q.q0 - p.q0,
        q.q1 - p.q1,
        q.q2 - p.q2,
        q.q3 - p.q3
    };

    return r;
}

void ScalarMultiplyQuaternion(Quaternion *q, float scalar) {
    q->q0 *= scalar;
    q->q1 *= scalar;
    q->q2 *= scalar;
    q->q3 *= scalar;
}

Quaternion MultiplyQuaternions(Quaternion q, Quaternion p) {

    Quaternion r = {
        ((q.q0 * p.q0) - (q.q1 * p.q1) - (q.q2 * p.q2) - (q.q3 * p.q3)),
        ((q.q0 * p.q1) - (q.q1 * p.q0) - (q.q2 * p.q3) - (q.q3 * p.q2)),
        ((q.q0 * p.q2) - (q.q1 * p.q3) - (q.q2 * p.q0) - (q.q3 * p.q1)),
        ((q.q0 * p.q3) - (q.q1 * p.q2) - (q.q2 * p.q1) - (q.q3 * p.q0))
    };

    return r;
}

Quaternion DivideQuaternions(Quaternion q, Quaternion p) {

    return MultiplyQuaternions(q, Get_QuaternionInverse(p));
}

void Rotate_Vector(float *vector, Quaternion q) {

    Quaternion P = { vector[0], vector[1], vector[2], vector[3] };
    Quaternion res = MultiplyQuaternions(MultiplyQuaternions(q, P), Get_QuaternionConjugate(q));
    vector[0] = res.q0;
    vector[1] = res.q1;
    vector[2] = res.q2;
    vector[3] = res.q3;
}

void RotateByAngle(Quaternion *q, float angle) {

    float sinCalculation = sin( (angle / 2) * DEGREE_TO_RAD);
    q->q0 = cos((angle / 2) * DEGREE_TO_RAD);
    q->q1 *= sinCalculation;
    q->q2 *= sinCalculation;
    q->q3 *= sinCalculation;
}


void NormalizeQuaternion(Quaternion *q) {

    float error = 1 - Get_QuaternionLength(*q);
    float denominator = sqrt((1 - error));
    q->q0 /= denominator;
    q->q1 /= denominator;
    q->q2 /= denominator;
    q->q3 /= denominator;

}

float Get_QuaternionLength(Quaternion q) {
    
    return sqrt( (q.q0 * q.q0) + (q.q1 * q.q1) + (q.q2 * q.q2) + (q.q3 * q.q3) );
}

Quaternion Get_QuaternionInverse(Quaternion q) {

    Quaternion r = Get_QuaternionConjugate(q);
    float length = Get_QuaternionLength(q);
    r.q0 /= length;
    r.q1 /= length;
    r.q2 /= length;
    r.q3 /= length;

    return r;
}

Quaternion Get_QuaternionConjugate(Quaternion q) {

    Quaternion r = {
        q.q0,
        q.q1 * -1.0,
        q.q2 * -1.0,
        q.q3 * -1.0
    };

    return r; 
}





/*
 * 			TO-DO
 * 			-----
 *  - 
 *
 *  - 
 *  
 *  - 
 *  */

