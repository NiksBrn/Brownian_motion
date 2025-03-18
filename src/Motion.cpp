#include "Motion.h"

// Детекция и обработка столкновений
void handleCollisions() {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            double dx = bodies[i].x - bodies[j].x;
            double dy = bodies[i].y - bodies[j].y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double min_distance = bodies[i].radius + bodies[j].radius;

            if (distance < min_distance) {
                // Нормаль столкновения
                double nx = dx / distance;
                double ny = dy / distance;

                // Относительная скорость
                double relative_velocity_x = bodies[i].vx - bodies[j].vx;
                double relative_velocity_y = bodies[i].vy - bodies[j].vy;

                // Импульс вдоль нормали
                double impulse = 2.0f * (relative_velocity_x * nx + relative_velocity_y * ny) /
                                (1.0f / bodies[i].mass + 1.0f / bodies[j].mass);

                // Обновление скоростей
                bodies[i].vx -= impulse / bodies[i].mass * nx;
                bodies[i].vy -= impulse / bodies[i].mass * ny;
                bodies[j].vx += impulse / bodies[j].mass * nx;
                bodies[j].vy += impulse / bodies[j].mass * ny;

                double overlap = min_distance - distance;
                bodies[i].x += nx * overlap / 2;
                bodies[i].y += ny * overlap / 2;
                bodies[j].x -= nx * overlap / 2;
                bodies[j].y -= ny * overlap / 2;
            }
        }
    }
}

void rungeKutta(RigidBody& body) {
  double k1x = body.vx;
  double k1y = body.vy;

  double k2x = body.vx + k1x * dt / 2;
  double k2y = body.vy + k1y * dt / 2;

  double k3x = body.vx + k2x * dt / 2;
  double k3y = body.vy + k2y * dt / 2;

  double k4x = body.vx + k3x * dt;
  double k4y = body.vy + k3y * dt;

  body.x = body.x + (k1x + 2 * k2x + 2 * k3x + k4x) * dt / 6;
  body.y = body.y + (k1y + 2 * k2y + 2 * k3y + k4y) * dt / 6;
}

void updateBodies() {
    double energy = 0;
    for (auto& body : bodies) {
        rungeKutta(body);

        // Отражение от стенок
        if (body.x < body.radius || body.x > box_size - body.radius) {
            body.vx = -body.vx;
        }
        if (body.y < body.radius || body.y > box_size - body.radius) {
            body.vy = -body.vy;
        }
        energy += body.mass * sqrt(body.vx * body.vx + body.vy * body.vy);
    }
    std::cout << energy << '\n';
}

