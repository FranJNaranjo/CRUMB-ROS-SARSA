/*
   CÓDIGO REALIZADO POR FRANCISCO JOSÉ NARANJO CAMPOS
               TRABAJO DE FIN DE GRADO
                      2019/2020
         ESCUELA DE INGENIERÍAS INDUSTRIALES
GRADO EN INGENIERÍA ROBÓTICA, ELECTRÓNICA Y MECATRÓNICA
*/

/*
Estea header define la clase "PhysicalHandle", que recogerá el comportamiento físico del robot crumb,
es decir, las acciones de movimiento de la base del CRUMB y la lectura de información del entorno (sensores, mapas, ...).
Con la información leida, determinará el estado actual del robot.
*/

//includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "rosgraph_msgs/Clock.h"
#include "math.h"
#include <gazebo_msgs/SetModelState.h>
#include <unistd.h>


using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Definición de clase physical_action                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PhysicalHandle
{
private:

#pragma region Internal Parameters 

    double _distanceMax;          //distancia maxima al target dist_max
    double _distanceDiv;          //tamaño de division de distancia
    double _orientationDiv;       //tamaño de division de la orientacion

    double _actionTime;           //tiempo de ejecución de la acción
    int _stateSize;            //longitud del vector de estados: state

    
    int _numActions;//número de acciones posibles                
    
    //número de posibles valores de la variable del esatdo i=0 (distancia a target)
    int _distanceLevels;
    //númro de posibles valores de la variable del estado i=1 (orientación a target)
    int _orientationLevels;
    //número de posibles valores de la variable del estado i=2 (tiempo de acción)
    int _timeLevels;
    //número de posibles estados (reales y fantasmas)
    int _numStates;
    

#pragma endregion

#pragma region Ros Handles
    //Variables manejadoras de ROS
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber subOdom;
    ros::ServiceClient clientPose;
#pragma endregion    

    //variable donde almacenamos la odometria del target
    std::vector<double> _target;

    //variable donde almacenamos la odometria del robot
    nav_msgs::Odometry _odom;

    //vector estado del robot de tamaño según parámetros
    std::vector<int> _state;

    //vector de opciones de tiempo
    std::vector<double> _timeOptions;

    //indice de vector de tempos
    int _timeOptionIndex;

public:
    //Método inicializador de la clase
    PhysicalHandle(double distMax, int distanceLevels, int orientationLevels, int timeLevels, double actionTime, int numAction, int stateSize, std::vector<double> timeOptions);      
    //Método destructor de la clase
    ~PhysicalHandle();  

#pragma region State Access Methods

    //Método de acceso público al vector estados
    int getState(int index)
    {
        return _state.at(index);
    }

    //Método para asignar un valor al vector de estados
    void setState(int index, int value)
    {
        _state.at(index) = value;
    }

    //Método de acceso público al tamaño del vector de estados
    int StateSize()
    {
        return _state.size();
    }   

#pragma endregion

#pragma region Methods Definition

    //Método que lee la posicion odométrica
    void odomCallback(const  gazebo_msgs::ModelStates::ConstPtr& msg);

    //Método para cambiar la pose del target en gazebo
    void setTargetMarkerPose(std::vector<double> targetPose);

    //Método que comanda un movimiento al CRUMB
    double actionComand(int action);

    //Método que calcula el estado actual del robot
    std::vector<int> lookState();

    //Método que convierte el vector del estado en un número entero (valor asociado al índice de la tabla)
    int stateToInt(std::vector<int> state);

    //Método que establece el target
    void setTarget(const  std::vector<double> destiny);

    //Método que proporciona un nuevo target en un radio de _distMax de distancia
    std::vector<double> getNewTarget(double distanceTarget);

    //Método que determina si se ha alcanzado el target
    bool isTargetAchieved();
    
    //Método que calcula el ángulo de rotación sobre el eje z a partir del cuaternión de la odometría
    double getOdomYaw(const nav_msgs::Odometry odom);

#pragma endregion

};


//////////////////////////////////////////////////////////////////////////////////////////////
//                          Definición de funciones                                         //
//////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Methods Specification

PhysicalHandle::PhysicalHandle(double distMax, int distanceLevels, int orientationLevels, int timeLevels, double actionTime, int numAction, int stateSize, std::vector<double> timeOptions)
{
    //inicialización de las variables    
    
    _distanceMax = distMax;                            
    _distanceLevels = distanceLevels;
    _orientationLevels = orientationLevels;
    _timeLevels = timeLevels;       
    _actionTime = actionTime;       
    _stateSize = stateSize; 
    _numActions = numAction;  
  
    _timeOptions.assign(timeOptions.size(),0);
    for(std::size_t i=0; i < _timeOptions.size();i++)
    {
        _timeOptions.at(i) = timeOptions.at(i);
    }

    _timeOptionIndex =0;
    for(std::size_t i = 0; i < _timeOptions.size();i++ )
    {
        if(_timeOptions.at(i)==_actionTime)
        {
            _timeOptionIndex = i;
        }
    }          

    _distanceDiv = _distanceMax / _distanceLevels;
    _orientationDiv = 2*3.1416 / _orientationLevels;

    if(_stateSize <= 2)
    {
       _numStates = pow(2, round(log2(_distanceLevels)) +round(log2(_orientationLevels)) ); 
    }
    else
    {
        _numStates = pow(2, round(log2(_distanceLevels)) +round(log2(_orientationLevels)) +round(log2(_timeLevels)) );
    }

    _state.assign( _stateSize,0);

    _target.assign(3,0);

    //Nos suscribimos a la odometría

    subOdom = n.subscribe("/gazebo/model_states", 1000,&PhysicalHandle::odomCallback,this);   //subcripción a odometría

    //Publicamos
    pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000); //publicación de comandos de acción

    //Creamos cliente del servicio
    clientPose = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

PhysicalHandle::~PhysicalHandle()
{
}

void PhysicalHandle::odomCallback(const  gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int crumbIndex = 1;
    for(size_t i =0 ; i< msg->name.size() ;i ++)
    {
        if(msg->name.at(i)=="crumb")
        {
            crumbIndex = i;
        }
    }
    _odom.pose.pose = msg -> pose.at(crumbIndex);
}

double PhysicalHandle::actionComand(int action)
{
    //función que comanda un movimiento al CRUMB
    // input: entero con la acción a relaizar
    // sin output

    //determinamos la accion que se nos solicita realizar según el entero "action"
    /*
    remain del significado del valor del entero action:
    default -> stop
    0 -> forward
    1 -> backward
    2 -> turn_left
    3 -> turn_right
    4 -> increase action time
    5 -> decrease action time
    */
    geometry_msgs::Twist sentMessage;
    double linear=0, angular=0;

    switch(action)
    {
        
        case 0://forward"
        {linear=0.3;angular=0;}
        break;
        case 1://backward"
        {linear=-0.3;angular=0;}
        break;
        case 2://turn_left"
        {linear=0;angular=0.9;}
        break;
        case 3://turn_right
        {linear=0;angular=-0.9;}
        break;
        case 4://increase action time
        {
            _timeOptionIndex++;
            if(_timeOptionIndex >= _timeOptions.size())
            { _timeOptionIndex = 0;}
            _actionTime = _timeOptions.at(_timeOptionIndex);
        }
        break;
        case 5://decrase action time
        {
            _timeOptionIndex--;
            if(_timeOptionIndex < 0)
            { _timeOptionIndex = _timeOptions.size()-1;}
            _actionTime = _timeOptions.at(_timeOptionIndex);
        }
        default:
        {linear=0;angular=0;}
    }

    double neededTime=0;
    if(action < 4)
    {
        sentMessage.linear.x=linear;
        sentMessage.angular.z=angular;

        ros::Time count = ros::Time::now();
        double start = double(count.toNSec())/1000000000;
        double stop = start;
        double uo = start;

        if(_actionTime < 0.5)
        {
            pub.publish(sentMessage);
            ros::Duration(_actionTime).sleep();
            count = ros::Time::now();
            stop = double(count.toNSec())/1000000000;
        }
        else if((_actionTime >= 0.5) && (_actionTime < 1))
        {
            while(((stop-start)<= _actionTime) && ros::ok())
            {
                pub.publish(sentMessage);
                ros::Duration(0.05).sleep();
                count = ros::Time::now();
                stop = double(count.toNSec())/1000000000;
            }
        }
        else if(_actionTime >= 1)
        {
            while(((stop-start)<= _actionTime) && ros::ok())
            {
                pub.publish(sentMessage);
                ros::Duration(0.1).sleep();
                count = ros::Time::now();
                stop = double(count.toNSec())/1000000000;
            }
        }
        std::cout << "\n"<< stop-start<<"\n";

        sentMessage.linear.x=0;
        sentMessage.angular.z=0;
        pub.publish(sentMessage);

        ros::spinOnce();
        neededTime =stop-start;
    }

    return neededTime;
}

std::vector<int> PhysicalHandle::lookState()
{
    //Método que determina el estado del robot
    ros::spinOnce();

    //Distancia a target
    double distance = sqrt( pow(_target.at(0) - _odom.pose.pose.position.x,2) +pow(_target.at(1) - _odom.pose.pose.position.y,2));
    if(distance >=_distanceMax)
    {
        setState(0,_distanceLevels-1);
    }
    else
    {
        setState(0,trunc(distance/_distanceDiv));
    }

    //Orientación al target
    double theta = atan2(_target.at(1)-_odom.pose.pose.position.y,_target.at(0)-_odom.pose.pose.position.x);
    theta = theta - 2*3.1416*floor(theta/(2*3.1416));
    double orientation = theta - getOdomYaw(_odom);


    orientation = orientation - 2*3.1416*floor(orientation/(2*3.1416));
    

    int orientationStateValue = trunc(orientation/_orientationDiv);
    if(orientationStateValue >= _orientationLevels)
        orientationStateValue = _orientationLevels-1;

    setState(1,orientationStateValue);

    //Nivel de tiempo
    if(_stateSize > 2)
    {
        setState(2,_timeOptionIndex);
    }
    
    return _state;
}

int PhysicalHandle::stateToInt(std::vector<int> state)
{
    //función que transforma el vector estado en un número entero asociado
    unsigned int sb=0;
    int d=0;

    if(_stateSize>2)
    {
        sb = state.at(2);
    }

    d = round(log2( _distanceLevels));
    sb = sb << d;
    sb = sb + state.at(0);

    d = round(log2( _orientationLevels));
    sb = sb << d;
    sb = sb + state.at(1);

    return sb;
}

void PhysicalHandle::setTarget(const  std::vector<double> destiny)
{
    //funcion que establece el target
    _target = destiny;

    //cambiamos la posición del modelo en gazebo
    setTargetMarkerPose(_target);
}

std::vector<double> PhysicalHandle::getNewTarget(double distanceTarget)
{
    //Método que proporciona un nuevo target aleatorio en un radio de distanceTarget de distancia

    //obtención de un ángulo
    double yaw = (rand()% 360);
    yaw = yaw *3.1416/180;

    //coordenadas x e y a despalzarse
    double x = distanceTarget * cos(yaw);
    double y = distanceTarget * sin(yaw);

    //hace spin para obtener posición del robot
    ros::spinOnce();

    //formar target
    std::vector<double> target;
    target.assign(3,0);
    target.at(0)= x + _odom.pose.pose.position.x;
    target.at(1) = y + _odom.pose.pose.position.y;
    yaw = (rand()% 360);
    yaw = yaw *3.1416/180;
    target.at(2) = yaw;

    std::cout << "Target ( x:" << target.at(0) <<"  y:"<< target.at(1)<<"  yaw:"<<target.at(2) <<" )\n";

    return target;
}

bool PhysicalHandle::isTargetAchieved()
{
    //Método que determina si se ha alcanzado el target
    bool targetAchieved = false;
    double distanceToTarget = sqrt( pow(_odom.pose.pose.position.x-_target.at(0),2) + pow(_odom.pose.pose.position.y-_target.at(1),2));
    double orientationToTarget = abs(getOdomYaw(_odom) - _target.at(2));
    if((_state.at(0)<1) )
    {
        targetAchieved = true;
    }
    
    return targetAchieved;
}

double PhysicalHandle::getOdomYaw(const nav_msgs::Odometry odom)
{
    //Método que calcula el ángulo de rotación sobre el eje z a partir del cuaternión de la odometría
    double x = odom.pose.pose.orientation.x;
    double y = odom.pose.pose.orientation.y;
    double z = odom.pose.pose.orientation.z;
    double w = odom.pose.pose.orientation.w;

    double psi;
    psi = atan2(2*(w*z+x*y),1-2*(pow(y,2)+pow(z,2)));
    psi = psi - 2*3.1416*floor(psi/(2*3.1416));

    return psi;
}

void PhysicalHandle::setTargetMarkerPose(std::vector<double> targetPose)
{
//Método para cambiar la pose del target en gazebo
gazebo_msgs::SetModelState modelPose;

modelPose.request.model_state.model_name="TargetMark5";
modelPose.request.model_state.pose.position.x = targetPose.at(0);
modelPose.request.model_state.pose.position.y = targetPose.at(1);
modelPose.request.model_state.pose.position.z = 0;
modelPose.request.model_state.pose.orientation.w = 1;
modelPose.request.model_state.pose.orientation.x = 0;
modelPose.request.model_state.pose.orientation.y = 0;
modelPose.request.model_state.pose.orientation.z = 0;
modelPose.request.model_state.twist.linear.x = 0.0;
modelPose.request.model_state.twist.linear.y = 0.0;
modelPose.request.model_state.twist.linear.z = 0.0;
modelPose.request.model_state.twist.angular.x = 0.0;
modelPose.request.model_state.twist.angular.y = 0.0;
modelPose.request.model_state.twist.angular.z = 0.0;
modelPose.request.model_state.reference_frame = "world";

clientPose.call(modelPose);

}
#pragma endregion


