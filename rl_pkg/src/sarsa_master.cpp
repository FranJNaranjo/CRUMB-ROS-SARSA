/*
   CÓDIGO REALIZADO POR FRANCISCO JOSÉ NARANJO CAMPOS
               TRABAJO DE FIN DE GRADO
                      2019/2020
         ESCUELA DE INGENIERÍAS INDUSTRIALES
GRADO EN INGENIERÍA ROBÓTICA, ELECTRÓNICA Y MECATRÓNICA
*/

/*
Este código implementa el nodo de ros que maneja y dirije el aprendizaje  y movimiento del robot CRUMB
*/

//includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string.h>

#include "PhysicalHandle.h"
#include "LearningHandle.h"



using namespace std;

#pragma region SIMULATION SETTINGS
    //PASOS TOTALES MÁXIMOS
    int MAX_STEP = 18000;

    //EPISODIOS TOTALES MÁXIMOS
    int NUM_EPISODES = 500;

    //PASOS MÁXIMOS POR EPISODIO
    int MAX_STEP_EPISODE = 250;

    //TIEMPO DE ACCIÓN (en segundos)
    // si está seleccionado el aprendizaje con elección de tiempos, este será el tiempo inicial
    double ACTION_TIME =0.1;  

    //DISTANCIA MÁXIMA DEL TARGET
    double MAX_TARGET = 0.5;

    //APRENDIZAJE CON ETAPAS DE DIFICULTAD (BOOSTRAPING)
    //si esta varibale es true, se realiza el boostraping
    bool BOOSTRAPING = true;

    //NÚMEOR DE EPISODIOS COMPLETADOS QUE FORMAN UNA ETAPA
    int PHASE = 100;

    //APRENDIZAJE CON ELECCIÓN DE TIEMPOS
    //si e sta variablees true, se realiza el aprendizaje con elección de tiempos
    bool TIME_SELECTION_RL = false;

    //Variable que determina si la inicialización de la tabla Q es por medio de un archivo externo
    bool INITILIZE_TAB = false;

#pragma endregion

#pragma region RL PARAMETERS

//PARÁMETRO DE DECUENTO O DISCOUNT RATE
double DISCOUNTING = 0.9;

//VALOR INICIAL DE LA TEMPERATURA
//estrategia de exploración de Boltzmann
double TEMP_INIT = 50;

//VELOCIDAD DE ENFRIAMIENTO
//estrategia de exploración de Boltzmann
double LAMBDA = 0.9999;

//PARÁMETRO EPSILON PARA E-GREEDY
double EPSILON = 0.3;

#pragma endregion


int main(int argc, char **argv)
{

#pragma region Tabs Settings

    double DISTANCE_MAX = 1.5;      //Distancia maxima al target dist_max
    int DISTANCE_LEVELS = 8;        //Niveles de disatncia al target (estado 0)
    int ORIENTATION_LEVELS = 8;     //Niveles de diferencia de orientación con el target (estado 1)
    int TIME_LEVELS = 4;            //Niveles de tiempo, número de posibles tiempos a escoger

    //Valor 0 indica que no aprende tiempo, valor 1 si lo aprende
    int ACTION_TIME_LEARNING = 0; 
    if(TIME_SELECTION_RL)
    {ACTION_TIME_LEARNING = 1;}

    

    //vector de las opciones de tiempo a elegir
    std::vector<double> TIME_OPTIONS;
    TIME_OPTIONS.assign(4,0);
    TIME_OPTIONS.at(0)=0.25;
    TIME_OPTIONS.at(1)=0.5;
    TIME_OPTIONS.at(2)=1.5;
    TIME_OPTIONS.at(3)=2;
    //longitud del vector de estados: state
    int STATE_SIZE = 2 + ACTION_TIME_LEARNING;

    //número de acciones posibles: 4 en el aprendizaje simple, 6 en el aprendizaje de tiempo
    int NUM_ACTIONS = 4 + ACTION_TIME_LEARNING*2;

#pragma endregion        
    
   //inicializamos nodo de ros sarsa master
    ros::init(argc, argv, "sarsa_master");

#pragma region Variables Declaration
    //declaración de variables

    //step o paso dentro de un episodio
    int currentStep = 1;

    //step o paso siguiente
    int nextStep = 2;

    //variable de almacenamiento del target
    std::vector<double> target;
    target.assign(3,0);

    //número entero asociado al estado, es decir, el estado
    int currentState;

    //número entero asociado al estado siguiente, es decir, el estado siguiente
    int nextState;

    //variable del accion a realizar en el paso st
    int currentAction;

    //variable del accion a realizar en el paso st_p
    int nextAction;

    //reward, recompensa 
    double reward;

    //espisodio
    int episode = 1;

    //vector para guardar y trabajar con el estado
    std::vector<int> stateVector;

    //variable que indica si el aprendizaje ha sido completado
    bool isCompleted = false;

    //variable que indica si el episodio ha terminado
    bool isAchieved = false;

    //variable con la distancia a la que se colocará el target
    double distanceTarget = 1;

    //contador de pasos máximos por episodio
    int countStep=1;

    //tiempo empleado en ejecutar la acción
    double neededTime;

    //numero total de targets alcanzados
    int numTargets=0;

    //declaramos objetos de las clases
    PhysicalHandle physicalHandle(DISTANCE_MAX, DISTANCE_LEVELS, ORIENTATION_LEVELS, TIME_LEVELS, ACTION_TIME, NUM_ACTIONS, STATE_SIZE, TIME_OPTIONS);
    LearningHandle learningHandle(DISTANCE_MAX, DISTANCE_LEVELS, ORIENTATION_LEVELS, TIME_LEVELS, ACTION_TIME, NUM_ACTIONS, STATE_SIZE, TIME_OPTIONS, INITILIZE_TAB, DISCOUNTING, TEMP_INIT, LAMBDA, EPSILON);

#pragma endregion

    //ALGORITMO SARSA
    
    //bucle episodio
    while ((episode <= NUM_EPISODES) && (currentStep < MAX_STEP) && (ros::ok()))//&& (isCompleted == false)  )
    {
        //Establecemos nuevo target
        if(!BOOSTRAPING)
        {
            distanceTarget = MAX_TARGET;
        }       
        else if(numTargets <= PHASE)
        {
            distanceTarget = 0.2;
        }
        else if(numTargets <= 2*PHASE)
        {
            distanceTarget = 0.2+(MAX_TARGET-0.2)/3;
        }
        else if(numTargets <= 3*PHASE)
        {
            distanceTarget = 0.2+2*(MAX_TARGET-0.2)/3;
        }
        else
        {
            distanceTarget = MAX_TARGET;
        }
        target = physicalHandle.getNewTarget(distanceTarget);
        physicalHandle.setTarget(target);

        //determinamos estado actual
        stateVector = physicalHandle.lookState();
        currentState = physicalHandle.stateToInt(stateVector);

        //determinamos accion a realizar
        currentAction = learningHandle.actionSelection(currentState);

        //bucle de cada paso
        while ((isAchieved == false) && (currentStep < MAX_STEP) && (ros::ok()))
        {

            //realizamos accion
            neededTime = physicalHandle.actionComand(currentAction);

            //determinamos nuevo estado
            stateVector = physicalHandle.lookState();
            nextState = physicalHandle.stateToInt(stateVector);

            //recibimos reward
            reward = learningHandle.getReward(stateVector);

            //determinamos accion a realizar
            nextAction = learningHandle.actionSelection(nextState);

            //actualizamos Q, V y Visit
            learningHandle.updateTabs(currentState,currentAction,reward,nextState,nextAction);

            //comprobamos si ha alcanzado el target
            isAchieved = physicalHandle.isTargetAchieved();

            //si se ha alcanzado el target, se cuenta
            if(isAchieved)
            {
                numTargets++;
            }

            //comprobamos si se ha superado el límite de pasos por episodio
            if(countStep >= MAX_STEP_EPISODE)
            {
                //se ha supeado el limite de pasos por episodio, otro epidosio

                isAchieved = true;
                countStep = 1;
                int notEpisode = -episode;
                //guardamos tablas en archivos, con episodio negativo
                learningHandle.writeFiles(notEpisode,currentStep,currentState,currentAction,reward,isAchieved,target,neededTime);
            }
            else
            {
                //no se ha superado el límite de pasos

                countStep++;
                //guardamos tablas en archivos
                learningHandle.writeFiles(episode,currentStep,currentState,currentAction,reward,isAchieved,target,neededTime);
            }

            if(isAchieved)
            {
                learningHandle.upadateLastVisit(nextState,nextAction);
            }

            //actualizamos estado actual y accion actual
            currentAction = nextAction;
            currentState = nextState;

            currentStep++; //siguiente paso
            std::cout <<"Episode: "<<episode<< "   Step: "<<currentStep<< "    Action:"<< currentAction << "\n";
            

        }//fin de bucle de paso
        learningHandle.saveEndEpiData(episode,currentState,currentAction);
        isAchieved = false;
        countStep=1;
        episode ++;
        isCompleted = learningHandle.endOfLearningChecker();

    }//fin bucle episodios

    //cerramos los archivos
    learningHandle.closeFiles();     

    return 0;
}

