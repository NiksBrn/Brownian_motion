#ifndef RIGIDBIDYSTRUCT_H
#define RIGIDBIDYSTRUCT_H
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <random>

// Параметры системы
extern const int num_bodies;  // Количество тел
extern const double dt;    // Шаг по времени
extern const double box_size;  // Размер области симуляции


// Структура для представления тела
struct RigidBody {
    double x, y;       // Позиция
    double vx, vy;     // Скорость
    double radius;     // Радиус
    double mass;       // Масса
};
extern std::vector<RigidBody> bodies;  // Вектор тел
void initBodies();


#endif //RIGIDBIDYSTRUCT_H
