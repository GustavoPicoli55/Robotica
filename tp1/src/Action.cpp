#include "Action.h"
#include <chrono>
#include "Utils.h"

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    const float FORWARD_VELOCITY = 1.4;
    const float TURN_VELOCITY = 0.5;

    const float LASER_OBSTACLE_THRESHOLD = 1.0;
    const float SONAR_OBSTACLE_THRESHOLD = 1.0;

    bool obstacle_detected = false;

    // Verifica obstáculos na frente com laser (±20º ao redor de 90º)
    for (int i = 70; i <= 110; ++i) {
        if (lasers[i] < LASER_OBSTACLE_THRESHOLD) {
            obstacle_detected = true;
            break;
        }
    }

    // Reforço: checa sonares frontais também (índices 2 a 5 costumam ser os centrais)
    for (int i = 2; i <= 5 && !obstacle_detected; ++i) {
        if (sonars[i] < SONAR_OBSTACLE_THRESHOLD) {
            obstacle_detected = true;
        }
    }

    if (obstacle_detected) {
        linVel = 0.0;
        angVel = TURN_VELOCITY;  // gira até desviar
    } else {
        linVel = FORWARD_VELOCITY;
        angVel = 0.0;            // segue em frente
    }
}



void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    static float previous_error = 0.0;
    static float integral = 0.0;
    static auto last_time = std::chrono::steady_clock::now();

    const float P = 0.5;
    const float I = 0.01;
    const float D = 2.0;

    const float FORWARD_VELOCITY = 1.0;

    float left = lasers[0];     // Lado esquerdo
    float right = lasers[180];  // Lado direito

    float error = left - right;

    // Limitando CTE a [-1, 1] para evitar mudanças bruscas
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;

    // Controle de tempo
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // Cálculo do PID
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;

    float pid = P * error + I * integral + D * derivative;

    // Segurança extra com laser e sonar frontais
    bool front_blocked = lasers[90] < 0.5;

    // Checagem complementar com sonares centrais
    for (int i = 3; i <= 4 && !front_blocked; ++i) {
        if (sonars[i] < 0.5) {
            front_blocked = true;
        }
    }

    if (front_blocked) {
        linVel = 0.0;
        angVel = 0.5;  // gira pra evitar colisão
    } else {
        linVel = FORWARD_VELOCITY;
        angVel = -pid;
    }
}



void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}

