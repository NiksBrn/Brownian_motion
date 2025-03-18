#ifndef MOTION_H
#define MOTION_H
#include "Rigidbody.h"
extern const int num_bodies;  // Количество тел
extern const double dt;    // Шаг по времени
extern const double box_size;  // Размер области симуляции
extern std::vector<RigidBody> bodies;  // Вектор тел

void handleCollisions();
void rungeKutta(RigidBody& body);
void updateBodies();

#endif //MOTION_H
