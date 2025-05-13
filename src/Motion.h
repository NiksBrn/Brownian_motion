#ifndef MOTION_H
#define MOTION_H
#include "RigidBody.h"
extern const int num_bodies;           // Количество тел
extern const double dt;                // Шаг по времени
extern const double box_size;          // Размер области симуляции
extern std::vector<RigidBody> bodies;  // Вектор тел
extern const float G;
extern const float softening;

void handleCollisions();
std::pair<float, float> computeForces(RigidBody &body);
void rungeKutta();
void energy();

#endif  // MOTION_H
