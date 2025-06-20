#include "Action.h"

#include "Utils.h"

//////////////////
/// CONSTRUTOR ///
//////////////////

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

///////////////////////////////
/// FUNCOES DE MOVIMENTACAO ///
///////////////////////////////

const double VELOCITY_ROT = 0.5; // rad/s
const double VELOCITY_FORW = 0.5; // m/s
const double ANGLE_THRESHOLD = 10.0; // degrees

void Action::followDirection(double angle)
{
     double angleAbs = fabs(angle);

    // Se o ângulo estiver dentro do limite, o robô pode seguir em frente com ajustes
    if (angleAbs <= ANGLE_THRESHOLD)
    {
        double factor = (ANGLE_THRESHOLD - angleAbs) / ANGLE_THRESHOLD;
        linVel = VELOCITY_FORW * factor;

        // Mantém um controle proporcional na rotação para corrigir a direção
        angVel = (angle / ANGLE_THRESHOLD) * VELOCITY_ROT;
    }
    else
    {
        // Fora do limite: apenas gira na direção do ângulo
        linVel = 0.0;
        angVel = VELOCITY_ROT * (angle > 0 ? 1 : -1);
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

void Action::stopRobot()
{
    linVel=0.0;
    angVel=0.0;
}

///////////////////////////
/// FUNCOES AUXILIARES  ///
///////////////////////////

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
        mc.mode=EXPLORE;
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

