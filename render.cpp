#include <cmath>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <fstream>

#include "render.h"

#define WIDTH 1000
#define HEIGHT 600

#define TIME_SCALE 1000
#define TIME_STEP 0.1
#define GRAPHS_TIME 8.0


// Global variables
bool isSphere = false;
double OldTime = -1, DeltaTime;
RigidBody rb;
Context context;

// Create 3d model of cube
void DrawCube(int size)
{
    // Check other modes: GL_POINTS GL_LINES GL_LINE_LOOP GL_LINE_STRIP GL_TRIANGLES GL_TRIANGLE_STRIP GL_TRIANGLE_FAN GL_QUAD_STRIP GL_POLYGON
    glBegin(GL_QUADS); // Begin drawing the color cube with 6 quads

    glColor3f(0.0f, size, 0.0f); // Green
    glVertex3f(size, size, -size);
    glVertex3f(-size, size, -size);
    glVertex3f(-size, size, size);
    glVertex3f(size, size, size);

    // Bottom face (y = -size)
    glColor3f(size, 0.5f, 0.0f); // Orange
    glVertex3f(size, -size, size);
    glVertex3f(-size, -size, size);
    glVertex3f(-size, -size, -size);
    glVertex3f(size, -size, -size);

    // Front face  (z = size)
    glColor3f(size, 0.0f, 0.0f); // Red
    glVertex3f(size, size, size);
    glVertex3f(-size, size, size);
    glVertex3f(-size, -size, size);
    glVertex3f(size, -size, size);

    // Back face (z = -size)
    glColor3f(size, size, 0.0f); // Yellow
    glVertex3f(size, -size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size, size, -size);
    glVertex3f(size, size, -size);

    // Left face (x = -size)
    glColor3f(0.0f, 0.0f, size); // Blue
    glVertex3f(-size, size, size);
    glVertex3f(-size, size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size, -size, size);

    // Right face (x = size)
    glColor3f(size, 0.0f, size); // Magenta
    glVertex3f(size, size, -size);
    glVertex3f(size, size, size);
    glVertex3f(size, -size, size);
    glVertex3f(size, -size, -size);
    glEnd(); // End of drawing color-cube
}

// Create model of sphere
void DrawSphere(float radius) {
    glColor3f(0.5f, 0.0f, 1.0f);
    glutSolidSphere(radius, 20, 20);
}

void DrawBackground() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, 1, 0, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_DEPTH_TEST); // Disable Z-test to keep the background behind everything

    glBegin(GL_QUADS);
    
    // Upper half (Air - gray)
    glColor3f(0.8f, 0.8f, 0.8f);
    glVertex2f(0.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);
    glVertex2f(1.0f, 0.5f);
    glVertex2f(0.0f, 0.5f);

    // Lower half (Water - blue)
    glColor3f(0.5f, 0.8f, 0.9f);
    glVertex2f(0.0f, 0.5f);
    glVertex2f(1.0f, 0.5f);
    glVertex2f(1.0f, 0.0f);
    glVertex2f(0.0f, 0.0f);

    glEnd();
    
    glEnable(GL_DEPTH_TEST); // Включаем обратно Z-тест

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

// Change size of window
void Reshape(int W, int H)
{
    // set camera model
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (double)W / H, 1, 500);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

// Create new frame in memory and draw it in window
void Display()
{
    // clear frame
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    DrawBackground();

    // solver step
    //double l = SolveEuler(rb, context, DeltaTime, OldTime);
    double l = SolveRungeKutta(rb, context, DeltaTime, OldTime);
    // double l = SolveHeuns(rb, context, DeltaTime, OldTime);
    //double l = SolveMidPoint(rb, context, DeltaTime, OldTime);
    //std::cout << "invariant: " << std::fixed << std::setprecision(15) << l << "\n";

    // create matrix in opengl stack
    glPushMatrix();

    // apply body translation
    glTranslated(rb.r.x, rb.r.y, rb.r.z);

    // apply body rotation
    rb.q = glm::normalize(rb.q);
    //glRotated(glm::degrees(acos(rb.q.w)) * 2, rb.q.x, rb.q.y, rb.q.z);
    glRotated(glm::degrees(2*atan2(sqrt(pow(rb.q.x, 2) + pow(rb.q.y, 2) + pow(rb.q.z, 2)), rb.q.w)), rb.q.x, rb.q.y, rb.q.z);

    // !!! Here transformations applied in reverse order because of OpenGL inner logic.
    // So we rotate around (0, 0, 0) first than translate to body position.
    // If your drawing function dont place body mass center in (0, 0, 0) -> you need to place one more glTranslated() here to fix it

    // draw body
    if (isSphere)
        DrawSphere(SIZE);
    else
        DrawCube(SIZE);

    // delete current matrix from opengl stack
    glPopMatrix();

    // put buffer from memory to screen
    glFlush();
    glutSwapBuffers();
}

// All events processed -> put new Display event in queue
void Idle()
{
    // update time
    long Time;
    if (OldTime == -1)
        OldTime = clock();
    Time = clock();
    DeltaTime = (double)(Time - OldTime) / CLOCKS_PER_SEC;
    DeltaTime *= 3.0;
    if (DeltaTime > 0.01) DeltaTime = 0.01; // max step
    OldTime = Time;

    // redraw frame
    glutPostRedisplay();
}

// Button click event
void Keyboard(unsigned char Key, int MouseX, int MouseY)
{
    if (Key == 27)
        exit(0);
    if (Key == 's' || Key == 'S') // press to change body
        isSphere = !isSphere;
}

// Функция для сохранения данных в файл
void saveData(const std::string& filename, const std::vector<double>& times, const std::vector<double>& energies) {
    std::ofstream dataFile(filename);
    if (!dataFile) {
        std::cerr << "Ошибка при открытии файла " << filename << "!\n";
        return;
    }
    for (size_t i = 0; i < times.size(); ++i) {
        dataFile << times[i] << " " << energies[i] << std::endl;
    }
    dataFile.close();
}

void plotGraphs() {
    std::ofstream gp("plot_commands.gp");
    gp << "set terminal png size 800,600\n";
    gp << "set output 'energy_comparison.png'\n";
    gp << "set title 'Comparison of Numerical Methods'\n";
    gp << "set xlabel 'Time (s)'\n";
    gp << "set ylabel 'Total Energy'\n";
    gp << "set grid\n";
    
    gp << "plot 'rk_energy.txt' using 1:2 with lines title 'Runge-Kutta' lw 2 lc rgb 'blue', "
          "'midpoint_energy.txt' using 1:2 with lines title 'Midpoint' lw 2 lc rgb 'green', "
          "'heuns_energy.txt' using 1:2 with lines title 'Heuns' lw 2 lc rgb 'orange', "
          "'euler_energy.txt' using 1:2 with lines title 'Euler' lw 2 lc rgb 'red'\n";
    gp.close();

    system("gnuplot plot_commands.gp");
    std::cout << "График сохранен в energy_comparison.png\n";
}

// Saving total energy and writing to the file
void GraphEnergy()
{
    RigidBody rb_rk, rb_midpoint, rb_euler, rb_heuns;
    context.M_inv = 1.0 / 6000000;
    context.I_inv = dmat3(0);

    rb_rk.r = rb_midpoint.r = rb_euler.r = rb_heuns.r = dvec3(0, 1.5 * SIZE, -200);
    rb_rk.q = rb_midpoint.q = rb_euler.q = rb_heuns.q = dquat(1, 0, 0, 0);
    rb_rk.l = rb_midpoint.l = rb_euler.l = rb_heuns.l = dvec3(0, 0, 0);
    rb_rk.L = rb_midpoint.L = rb_euler.L = rb_heuns.L = dvec3(5000, 5000, 0);

    std::vector<double> times, rk_total_e, midpoint_total_e, euler_total_e, heuns_total_e;
    for (double t = 0; t <= GRAPHS_TIME; t += TIME_STEP) {
        times.push_back(t);
        double rk_energy = SolveRungeKutta(rb_rk, context, TIME_STEP, t);
        double midpoint_energy = SolveMidPoint(rb_midpoint, context, TIME_STEP, t);
        double euler_energy = SolveEuler(rb_euler, context, TIME_STEP, t);
        double heuns_energy = SolveHeuns(rb_heuns, context, TIME_STEP, t);

        rk_total_e.push_back(rk_energy);
        midpoint_total_e.push_back(midpoint_energy);
        euler_total_e.push_back(euler_energy);
        heuns_total_e.push_back(heuns_energy);
    }

    saveData("rk_energy.txt", times, rk_total_e);
    saveData("midpoint_energy.txt", times, midpoint_total_e);
    saveData("euler_energy.txt", times, euler_total_e);
    saveData("heuns_energy.txt", times, heuns_total_e);

    plotGraphs();
}

// Start all calculations and drawing
void Run(int argc, char *argv[])
{
    context.M_inv = 1.0 / 6000000;
    context.I_inv = dmat3(0);
    
    if (isSphere) {
        // sphere inertia tensor
        for (int i = 0; i < 3; i++)
            context.I_inv[i][i] = 2.0 / (5 / context.M_inv) * SIZE * SIZE; // SIZE - radius of the sphere
    }
    else {
        // cube inertia tensor
        for (int i = 0; i < 3; i++)
            context.I_inv[i][i] = 1.0 / (12 / context.M_inv) * 2 * SIZE * SIZE;
    }

    // r_0
    rb.r = dvec3(0, 1.5*SIZE, -200);
    // q_0
    rb.q = dquat(1, 0, 0, 0);
    // l_0
    rb.l = dvec3(0, 0, 0);
    // L_0
    rb.L = dvec3(5000, 5000, 0);

    // initialization
    glutInit(&argc, argv);

    // Request double buffered true color window with Z-buffer
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // Set size of window
    glutInitWindowSize(WIDTH, HEIGHT);
    // Windows position on screen
    glutInitWindowPosition(0, 0);
    // Creating window with name
    glutCreateWindow("RIGIT BODY SIMULATION");

    // Set functions for GLUT loop
    glutReshapeFunc(Reshape);
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glutKeyboardFunc(Keyboard);

    // Enable 3D mode
    glEnable(GL_DEPTH_TEST);

    // Start infinite loop
    glutMainLoop();
}
