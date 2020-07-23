/*
   CÓDIGO REALIZADO POR FRANCISCO JOSÉ NARANJO CAMPOS
               TRABAJO DE FIN DE GRADO
                      2019/2020
         ESCUELA DE INGENIERÍAS INDUSTRIALES
GRADO EN INGENIERÍA ROBÓTICA, ELECTRÓNICA Y MECATRÓNICA
*/

/*
Código de un nodo para testear de forma manual que se calculan correctamente los estados relacionados con el target.
Sirve como control manual del robot y del header PhysicalHandle.h.
*/

#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "PhysicalHandle.h"
#include "LearningHandle.h"



using namespace std;


int main(int argc, char **argv)
{
#pragma region Settings

    double DISTANCE_MAX = 1.5;       //Distancia maxima al target dist_max
    int DISTANCE_LEVELS = 8;        //Niveles de disatncia al target (estado 0)
    int ORIENTATION_LEVELS = 8;     //Niveles de diferencia de orientación con el target (estado 1)
    int TIME_LEVELS = 4;            //Niveles de tiempo, número de posibles tiempos a escoger
    //Valor 0 indica que no aprende tiempo, valor 1 si lo aprende
    int ACTION_TIME_LEARNING = 0; 

    //tiempo de ejecución de la acción en segundos, o tiempo inicial en un aprendizaje de tiempo
    double ACTION_TIME =1;  

    //vector de las opciones de tiempo a elegir
    std::vector<double> TIME_OPTIONS ;//= {0.25, 0.5,1.5 ,2};
    TIME_OPTIONS.assign(4,0);
    TIME_OPTIONS.at(0)=0.25;
    TIME_OPTIONS.at(1)=0.5;
    TIME_OPTIONS.at(2)=1.5;
    TIME_OPTIONS.at(3)=2;
    //longitud del vector de estados: state
    int STATE_SIZE = 2 + ACTION_TIME_LEARNING;

    //número de acciones posibles: 4 en el aprendizaje simple, 6 en el aprendizaje de tiempo
    int NUM_ACTIONS = 4;
    if(ACTION_TIME_LEARNING = 1)
    {
        NUM_ACTIONS = 6;
    }

    //máximo de iteracciones
    int MAX_STEP = 18000;

    //número de episodios
    int NUM_EPISODES = 500;

    //Variable que determina si la inicialización de la tabla Q es por medio de un archivo externo
    bool INITILIZE_TAB = false;

    //Variable del número máximo de pasos por episodios
    int MAX_STEP_EPISODE = 250;
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
    

    //declaramos objetos de las clases
    PhysicalHandle physicalHandle(DISTANCE_MAX, DISTANCE_LEVELS, ORIENTATION_LEVELS, TIME_LEVELS, ACTION_TIME, NUM_ACTIONS, STATE_SIZE, TIME_OPTIONS);
    //LearningHandle learningHandle(DISTANCE_MAX, RANGE_MAX, RANGE_MIN, NUM_SECTOR, DISTANCE_LEVELS, ORIENTATION_LEVELS, OBSTACLE_LEVELS, ACTION_TIME, NUM_ACTIONS, INITILIZE_TAB);
#pragma endregion

    char in;
    int estado;
    //establecemos un target inicial
    //target = physicalHandle.getNewTarget(distanceTarget);
    target.assign(3,0);
    physicalHandle.setTarget(target);

    while((in != 'c')&&(in != 'C'))
    {
        std::cout << "\n \n \n \n \n ";
        std::cout << "\n Target->    x: "<<target.at(0)<<"   y: "<<target.at(1)<<" \n";
        std::cout << "\nAcción a realizar:\n" << "- Mover adelante/atrás: W/S\n" << "- Girar izquierda/derecha: A/D\n";
        std::cout << "- Establecer nuevo target: T\n" << "- Cerrar: C\n" << "    Tecla pulsada: ";

        std::cin >> in;

        switch (in)
        {
        case 'w':
            physicalHandle.actionComand(0);
            std::cout << "\nMoviendo hacia adelante\n";
            break;
        case 'W':
            physicalHandle.actionComand(0);
            std::cout << "\nMoviendo hacia adelante\n";
            break;
        case 's':
            physicalHandle.actionComand(1);
            std::cout << "\nMoviendo hacia atrás\n";
            break;
        case 'S':
            physicalHandle.actionComand(1);
            std::cout << "\nMoviendo hacia atrás\n";
            break;
        case 'a':
            physicalHandle.actionComand(2);
            std::cout << "\nGirando a la izquierda\n";
            break;
        case 'A':
            physicalHandle.actionComand(2);
            std::cout << "\nGirando a la izquierda\n";
            break;
        case 'd':
            physicalHandle.actionComand(3);
            std::cout << "\nGirando a la derecha\n";
            break;
        case 'D':
            physicalHandle.actionComand(3);
            std::cout << "\nGirando a la derecha\n";
            break;
        case 't':
            std::cout << "\nDistancia al nuevo target (m):";
            std::cin>>distanceTarget;
            target = physicalHandle.getNewTarget(distanceTarget);
            physicalHandle.setTarget(target);
            break;
        case 'T':
            std::cout << "\nDistancia al nuevo target (m):";
            std::cin>>distanceTarget;
            target = physicalHandle.getNewTarget(distanceTarget);
            physicalHandle.setTarget(target);
            break;
        case 'c':
            std::cout << "\nCERRANDO\n";
            break;
        case 'C':
            std::cout << "\nCERRANDO\n";
            break;
        default:
            break;
        }

        if((in != 'c')&&(in != 'C'))
        {
            stateVector = physicalHandle.lookState();
            estado = physicalHandle.stateToInt(stateVector);
            std::cout << "\n Estado " << estado<<":\n";
            std::cout << "- Nivel distancia a target: "<< stateVector.at(0)<< "\n";
            std::cout << "- Nivel angulo con el target: "<< stateVector.at(1) << "\n";

        }
    }

    return 0;
}

