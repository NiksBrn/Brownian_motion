#ifndef RENDER_H
#define RENDER_H

#include <GL/freeglut.h>
#include "Motion.h"
#include "Rigidbody.h"
extern const int num_bodies;  // Количество тел
extern const double dt;    // Шаг по времени
extern const double box_size;  // Размер области симуляции
extern std::vector<RigidBody> bodies;  // Вектор тел
void drawBodies();
void display();
void update(int value);
void initGL();
void Keyboard(unsigned char Key, int MouseX, int MouseY);
void Run(int argc, char** argv);


#endif //RENDER_H
