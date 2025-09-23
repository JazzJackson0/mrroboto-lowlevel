#include "../include/Quaternions.h"


Quaternion addQuaternions(Quaternion q, Quaternion p) {
   
    Quaternion r = { 
        q.q0 + p.q0,
        q.q1 + p.q1,
        q.q2 + p.q2,
        q.q3 + p.q3
    };

    return r;
}

Quaternion subtractQuaternions(Quaternion q, Quaternion p) {
    Quaternion r = { 
        q.q0 - p.q0,
        q.q1 - p.q1,
        q.q2 - p.q2,
        q.q3 - p.q3
    };

    return r;
}

void scalarMultiplyQuaternion(Quaternion *q, float scalar) {
    q->q0 *= scalar;
    q->q1 *= scalar;
    q->q2 *= scalar;
    q->q3 *= scalar;
}

Quaternion multiplyQuaternions(Quaternion q, Quaternion p) {

    Quaternion r = {
        ((q.q0 * p.q0) - (q.q1 * p.q1) - (q.q2 * p.q2) - (q.q3 * p.q3)),
        ((q.q0 * p.q1) - (q.q1 * p.q0) - (q.q2 * p.q3) - (q.q3 * p.q2)),
        ((q.q0 * p.q2) - (q.q1 * p.q3) - (q.q2 * p.q0) - (q.q3 * p.q1)),
        ((q.q0 * p.q3) - (q.q1 * p.q2) - (q.q2 * p.q1) - (q.q3 * p.q0))
    };

    return r;
}

Quaternion divideQuaternions(Quaternion q, Quaternion p) {

    return multiplyQuaternions(q, getQuaternionInverse(p));
}

void rotateVector(float *vector, Quaternion q) {

    Quaternion P = { vector[0], vector[1], vector[2], vector[3] };
    Quaternion res = multiplyQuaternions(multiplyQuaternions(q, P), getQuaternionConjugate(q));
    vector[0] = res.q0;
    vector[1] = res.q1;
    vector[2] = res.q2;
    vector[3] = res.q3;
}

void rotateByAngle(Quaternion *q, float angle) {

    float sinCalculation = sin( (angle / 2) * DEGREE_TO_RAD);
    q->q0 = cos((angle / 2) * DEGREE_TO_RAD);
    q->q1 *= sinCalculation;
    q->q2 *= sinCalculation;
    q->q3 *= sinCalculation;
}


void normalizeQuaternion(Quaternion *q) {

    float error = 1 - getQuaternionLength(*q);
    float denominator = sqrt((1 - error));
    q->q0 /= denominator;
    q->q1 /= denominator;
    q->q2 /= denominator;
    q->q3 /= denominator;

}

float getQuaternionLength(Quaternion q) {
    
    return sqrt( (q.q0 * q.q0) + (q.q1 * q.q1) + (q.q2 * q.q2) + (q.q3 * q.q3) );
}

Quaternion getQuaternionInverse(Quaternion q) {

    Quaternion r = getQuaternionConjugate(q);
    float length = getQuaternionLength(q);
    r.q0 /= length;
    r.q1 /= length;
    r.q2 /= length;
    r.q3 /= length;

    return r;
}

Quaternion getQuaternionConjugate(Quaternion q) {

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

