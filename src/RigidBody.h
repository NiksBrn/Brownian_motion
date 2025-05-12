#ifndef RIGIDBIDYSTRUCT_H
#define RIGIDBIDYSTRUCT_H
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>
#include <vector>

// Параметры системы
extern const int num_bodies;   // Количество тел
extern const double dt;        // Шаг по времени
extern const double box_size;  // Размер области симуляции
extern const float G;  
extern const float softing;

// Структура для представления тела
struct RigidBody {
  double x, y;     // Позиция
  double vx, vy;   // Скорость
  double radius;   // Радиус
  double mass;     // Масса
  double r, g, b;  // Цвет
};
extern std::vector<RigidBody> bodies;  // Вектор тел
void initBodies();

#endif  // RIGIDBIDYSTRUCT_H
