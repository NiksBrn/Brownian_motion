#include "Motion.h"

// Детекция и обработка столкновений
void handleCollisions() {
  for (size_t i = 0; i < bodies.size(); ++i) {
    // Отражение от стенок
    if (bodies[i].x < bodies[i].radius ||
        bodies[i].x > box_size - bodies[i].radius) {
      bodies[i].vx = -bodies[i].vx;
    }
    if (bodies[i].y < bodies[i].radius ||
        bodies[i].y > box_size - bodies[i].radius) {
      bodies[i].vy = -bodies[i].vy;
    }

    for (size_t j = i + 1; j < bodies.size(); ++j) {
      float dx = bodies[i].x - bodies[j].x;
      float dy = bodies[i].y - bodies[j].y;
      float distance = std::sqrt(dx * dx + dy * dy);
      float min_distance = bodies[i].radius + bodies[j].radius;

      if (distance < min_distance) {
        // Нормаль столкновения
        float nx = dx / distance;
        float ny = dy / distance;

        // Относительная скорость
        float relative_velocity_x = bodies[i].vx - bodies[j].vx;
        float relative_velocity_y = bodies[i].vy - bodies[j].vy;

        // Импульс вдоль нормали
        float impulse = 2.0f *
                        (relative_velocity_x * nx + relative_velocity_y * ny) /
                        (1.0f / bodies[i].mass + 1.0f / bodies[j].mass);

        // Обновление скоростей
        bodies[i].vx -= impulse / bodies[i].mass * nx;
        bodies[i].vy -= impulse / bodies[i].mass * ny;
        bodies[j].vx += impulse / bodies[j].mass * nx;
        bodies[j].vy += impulse / bodies[j].mass * ny;

        // Разделение тел
        float overlap = min_distance - distance;
        bodies[i].x += nx * overlap / 2;
        bodies[i].y += ny * overlap / 2;
        bodies[j].x -= nx * overlap / 2;
        bodies[j].y -= ny * overlap / 2;
      }
    }
  }
}

std::pair<float, float> computeForces(Body& body) {
  float fx = 0.0f, fy = 0.0f;

  for (const auto& other : bodies) {
    if (&body == &other) continue;

    float dx = other.x - body.x;
    float dy = other.y - body.y;
    float distSq = dx * dx + dy * dy + softening * softening;
    float dist = sqrt(distSq);
    float invDist3 = 1.0f / (dist * distSq);

    fx += G * other.mass * dx * invDist3;
    fy += G * other.mass * dy * invDist3;
  }
  return std::make_pair(fx, fy);
}

void rungeKutta() {
  for (auto& body : bodies) {
    float origin_x = body.x;
    float origin_y = body.y;
    float origin_vx = body.vx;
    float origin_vy = body.vy;
    auto [fx, fy] = computeForces(body);
    // k1 = f(t, x)
    float k1vx = fx;
    float k1vy = fy;
    float k1x = body.vx;
    float k1y = body.vy;
    // k2 = f(t + h/2, x + h/2*k1)
    body.x = origin_x + k1x * dt / 2;
    body.y = origin_y + k1y * dt / 2;
    body.vx = origin_vx + k1vx * dt / 2;
    body.vy = origin_vy + k1vy * dt / 2;
    auto [fx2, fy2] = computeForces(body);
    float k2vx = fx2;
    float k2vy = fy2;
    float k2x = body.vx;
    float k2y = body.vy;
    // k3 = f(t + h/2, x + h/2*k2)
    body.x = origin_x + k2x * dt / 2;
    body.y = origin_y + k2y * dt / 2;
    body.vx = origin_vx + k2vx * dt / 2;
    body.vy = origin_vy + k2vy * dt / 2;
    auto [fx3, fy3] = computeForces(body);
    float k3vx = fx3;
    float k3vy = fy3;
    float k3x = body.vx;
    float k3y = body.vy;
    // k4 = f(t + h, x + h*k3)
    body.x = origin_x + k3x * dt;
    body.y = origin_y + k3y * dt;
    body.vx = origin_vx + k3vx * dt;
    body.vy = origin_vy + k3vy * dt;
    auto [fx4, fy4] = computeForces(body);
    float k4vx = fx4;
    float k4vy = fy4;
    float k4x = body.vx;
    float k4y = body.vy;
    //
    body.vx = origin_vx + dt * (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6;
    body.vy = origin_vy + dt * (k1vy + 2 * k2vy + 2 * k3vy + k4vy) / 6;
    body.x = origin_x + dt * (k1x + 2 * k2x + 2 * k3x + k4x) / 6;
    body.y = origin_y + dt * (k1y + 2 * k2y + 2 * k3y + k4y) / 6;
  }
}

void energy() {
  float total_energy = 0.0f;
  float kinetic_energy = 0.0f;
  float potential_energy = 0.0f;
  for (auto& body : bodies) {
    kinetic_energy +=
        0.5f * body.mass * (body.vx * body.vx + body.vy * body.vy);
  }
  for (size_t i = 0; i < bodies.size(); ++i) {
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      potential_energy +=
          -G * bodies[i].mass * bodies[j].mass /
          (std::sqrt((bodies[i].x - bodies[j].x) * (bodies[i].x - bodies[j].x) +
                     (bodies[i].y - bodies[j].y) *
                         (bodies[i].y - bodies[j].y)));
    }
  }
  total_energy = kinetic_energy + potential_energy;
  std::cout << "Total energy: " << total_energy << std::endl;
}