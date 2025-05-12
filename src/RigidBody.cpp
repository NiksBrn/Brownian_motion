#include "RigidBody.h"

const int num_bodies = 50;      // Количество тел
const double dt = 0.01f;        // Шаг по времени
const double box_size = 10.0f;  // Размер области симуляции
const float G = 0.01f;           // Гравитационная постоянная
const float softing = 0.05f;    // Фактор мягкости для учета сокращения радиусов
std::vector<RigidBody> bodies;  // Вектор тел

// Инициализация тел
void initBodies() {
  // Движок для рандома
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.1, 0.6);
  std::uniform_real_distribution<> dis2(1.5, box_size - 1.5);
  srand(static_cast<unsigned int>(time(0)));

  for (int i = 0; i < num_bodies; ++i) {
    RigidBody body;
    // Координаты
    body.x = static_cast<double>(dis2(gen));
    body.y = static_cast<double>(dis2(gen));
    // Вектора
    body.vx = static_cast<double>(rand()) / RAND_MAX * 2 - 1;
    body.vy = static_cast<double>(rand()) / RAND_MAX * 2 - 1;
    // Радиус и масса
    body.radius = dis(gen);
    body.mass = static_cast<double>(1.0f * body.radius);
    // Цвет
    body.r = (rand() % 100) / 100.0f;
    body.g = (rand() % 100) / 100.0f;
    body.b = (rand() % 100) / 100.0f;

    bodies.push_back(body);
  }
}