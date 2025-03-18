#include "Render.h"

// Отрисовка тел
void drawBodies() {
    for (const auto& body : bodies) {
        glBegin(GL_TRIANGLE_FAN);
        glColor3f(1.0f, 0.0f, 0.0f);  // Красный цвет
        glVertex2f(body.x, body.y);  // Центр круга
        for (int i = 0; i <= 360; i += 10) {
            double angle = i * 3.14159f / 180.0f;
            double x = body.x + body.radius * std::cos(angle);
            double y = body.y + body.radius * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    drawBodies();
    glutSwapBuffers();
}

void update(int value) {
    updateBodies();
    handleCollisions();
    glutPostRedisplay();
    glutTimerFunc(16, update, 0);
}

void initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Черный фон
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, box_size, 0, box_size);  // Область симуляции
}

void Keyboard(unsigned char Key, int MouseX, int MouseY)
{
    if (Key == 27)
        exit(0);
}

void Run(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("2D Brownian Motion with Collisions");

    initGL();
    initBodies();

    glutDisplayFunc(display);
    glutTimerFunc(0, update, 0);
    glutKeyboardFunc(Keyboard);

    glutMainLoop();
}