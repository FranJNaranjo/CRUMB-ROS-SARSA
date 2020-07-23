/*
   CÓDIGO REALIZADO POR FRANCISCO JOSÉ NARANJO CAMPOS
               TRABAJO DE FIN DE GRADO
                      2019/2020
         ESCUELA DE INGENIERÍAS INDUSTRIALES
GRADO EN INGENIERÍA ROBÓTICA, ELECTRÓNICA Y MECATRÓNICA
*/

/*
En este header se define la clase "LearningHandle", la cual define el proceos y las funciones
encargadas del aprendizaje por refuerzo ( tabla Q, tabla de visitas, etc), de la función adjudicadora
de la recompensa (reward) y de la función que selecciona la acción a realizar.
*/

//includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <string.h>


using namespace std;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Definición de clase learning                                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

class LearningHandle
{
private:

#pragma region Internal Parameters
    
    double   _distanceMax;          //distancia maxima al target dist_max
    double   _distanceDiv;          //tamaño de division de distancia
    double   _orientationDiv;       //tamaño de division de la orientacion

    double   _actionTime;           //tiempo de ejecución de la acción
    int      _stateSize;            //longitud del vector de estados: state   

    int   _numActions;//número de acciones posibles                
    
    //número de posibles valores de la variable del esatdo i=0 (distancia a target)
    int   _distanceLevels;
    //númro de posibles valores de la variable del estado i=1 (orientación a target)
    int   _orientationLevels;
    //número de posibles valores de la variable del estado i=2 (tiempo de acción)
    int _timeLevels;
    //número de posibles estados (reales y fantasmas)
    int   _numStates;

    //paraetro alpha, step-size
    double alpha;
    //parámetro discountign rate o gamma
    double _discounting;

    //parámetro de temperatura de la distribución boltzmann
    double temp;
    double _lambda;

    //parámetro para e-greedy de exploración-explotación _epsilon
    double _epsilon;

#pragma endregion   

#pragma region Files definition
    //archivos para guardar tablas
    fstream QFile;
    fstream VFile;
    fstream VisitFile;
    fstream RecordFile;
    fstream EpisodeFile;
    fstream TargetFile;
    fstream QFinal;
    fstream EndEpiInfo;
#pragma endregion

#pragma region Tabs definition
    
    //tabla Q(s,a) estimated state-action value
    std::vector<std::vector<double> > Q;

    //vector V(s) estimated state value
    std::vector<double> V;

    //vector V(s)_k-1 estimated state value at previous step
    std::vector<double> PreviousV;

    //vector de visitas visit(s)
    std::vector<std::vector<double> > Visit;

    //vector de la probabilidad de elección de cada acción
    std::vector<double> Prob;


#pragma endregion

public:
    LearningHandle(double distMax, int distanceLevels, int orientationLevels, int timeLevels, double actionTime, int numAction, int stateSize, std::vector<double> timeOptions, bool initializeTab, double discounting, double temp_init, double lambda, double epsilon);
    ~LearningHandle();

#pragma region Methods definition

    //Método que inicializa la tabla Q desde un archivo externo llamado QIni.txt
    void readExternalTab();

    //Método que actualiza la tabla Q, V y visit, según el algoritma SARSA
    int updateTabs(int s,int a,double r,int sp,int ap);

    //Método que selecciona la acción por el método de Boltzmann
    int actionSelection(int s);

    //Método que selecciona la acción por el método e-greedy
    int actionSelectionE(int s);

    //Método que deveulve la recompensa, reward, según el estado
    double getReward(std::vector<int> state);

    //Método que devuelve true si ya se ha completado el aprendizaje
    bool endOfLearningChecker();

    //función que escribe las tablas en archivos txt
    void writeFiles(int episode, int s, int state, int action, double reward, bool endOfEpisode, std::vector<double> target, double neededTime);

    //función para cerrar archivos
    void closeFiles();

    //método que limita el número de caracteres a 10
    double limitNum(double num);

    //método que actualiza la tabla visitas al final del episodio
    void upadateLastVisit(int s, int a);

    //método para guardar datos extras sobre el final de episodios
    void saveEndEpiData(int episode, int sp, int ap);
    

#pragma endregion

};

//////////////////////////////////////////////////////////////////////////////////////////////
//                          Definición de funciones                                         //
//////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Methods Specification

LearningHandle::LearningHandle(double distMax, int distanceLevels, int orientationLevels, int timeLevels, double actionTime, int numAction, int stateSize, std::vector<double> timeOptions, bool initializeTab, double discounting, double temp_init, double lambda, double epsilon)
{
    //inicialización de las variables    
    
    _distanceMax = distMax;                            
    _distanceLevels = distanceLevels;
    _orientationLevels = orientationLevels;
    _timeLevels = timeLevels;       
    _actionTime = actionTime;
    _stateSize = stateSize;        
    _numActions = numAction;  


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
    


    //inicialización de la tabla Q(s,a)
    Q.assign( _numStates,vector<double>( _numActions,0));

    if(initializeTab == true)
    {
        readExternalTab();
    }

    //inicialización del vector V(s) y V(s)-1
    V.assign( _numStates,0);
    PreviousV.assign( _numStates,0);

    //inicialización del vector visitas visit(s)
    Visit.assign( _numStates,vector<double>( _numActions,0));

    //inicialización del vectorde probabilidades de acción
    Prob.assign(_numActions,1/_numActions);

    //crear y abrir archivos txt
    QFile.open("QTab.txt",fstream::out);
    VFile.open("VTab.txt",fstream::out);
    VisitFile.open("visitTab.txt",fstream::out);
    RecordFile.open("RecordTab.txt",fstream::out);
    EpisodeFile.open("EpisodeCompletation.txt",fstream::out);
    TargetFile.open("Target.txt",fstream::out);
    QFinal.open("QFinal.txt",fstream::out);
    EndEpiInfo.open("EndEpiInfo.txt",fstream::out);

    //inicialización parámetros de aprendizaje
    _discounting = discounting;
    temp = temp_init;
    _lambda = lambda;
    _epsilon = epsilon;

}

LearningHandle::~LearningHandle()
{
}


void LearningHandle::readExternalTab()
{
    //Método que inicializa la tabla Q desde un archivo externo llamado QIni.txt
    ifstream tabQ;
    ifstream tabV;

    tabQ.open("QIni.txt");
    if(!tabQ)
        std::cout << "\n Error abriendo la tabla Q inicializadora\n";

    tabV.open("VIni.txt");
    if(!tabV)
        std::cout << "\n Error abriendo la tabla Q inicializadora\n";    

    double aux =0;
    for(size_t i=0;i < Q.size();i++)
    {
        for(size_t j=0;j<Q.at(i).size();j++)
        {
            tabQ >> aux;
            Q.at(i).at(j) = aux;
        }
    }

    aux =0;
    for(size_t i=0;i < V.size();i++)
    {
        tabV >> aux;
        V.at(i) = aux;
    }

    tabQ.close();
    tabV.close();
}

int LearningHandle::updateTabs(int s,int a,double r,int sp,int ap)
{
    //Función que actualiza las tablas Q(s,a), V(s) y visit(a)
    
    //Cálculo del alpha o step-size para este caso
    if(Visit.at(s).at(a) == 0)
    {
        alpha = 1;
    } 
    else
    {
        alpha = 1/Visit.at(s).at(a);
    }
    

    //Actualización de tabla Q(s,a)
    if( (s < Q.size()) && (sp < Q.size()) )
    {
        if( (a < Q.at(s).size()) && (ap < Q.at(sp).size()) )
        {
            Q.at(s).at(a) = Q.at(s).at(a) + alpha*(r+_discounting*Q.at(sp).at(ap)-Q.at(s).at(a));
        }
        else
        {
            std::cout << "\n Desbordamiento de Q("<< s << " o "<< sp <<").(x) al intentar acceder a x = "<< a <<" o "<< ap <<"\n";
            std::cout << "Tamaños de Q(s) y Q(sp) = "<< Q.at(s).size() << " y "<< Q.at(sp).size()<<" \n";
        }
    }
    else
    {
        std::cout << "\n Desbordamiento de Q(x) al intentar acceder a x = "<< s <<" o "<< sp <<"\n";
        std::cout << "Tamaño de Q = "<< Q.size()<<" \n";
        }
      
    //Actualizamos tabla de visitas
    if(s < Visit.size())
    {
        if(a < Visit.at(s).size())
        {
            Visit.at(s).at(a)++;
        }
        else
        {
            std::cout << "\n Desbordamiento de visit("<< s <<").(x) al intentar acceder a x = "<< a <<"\n";
            std::cout << "Tamaño de visit = "<< Visit.at(s).size()<<" \n";        
        }     
    }
    else
    {
        std::cout << "\n Desbordamiento de visit(x) al intentar acceder a x = "<< s <<"\n";
        std::cout << "Tamaño de visit = "<< Visit.size()<<" \n";
    }
     

    //Actualización de V(s)
    PreviousV = V;
    if((s < V.size()) && (sp < V.size()) )
    {
        double max=0;
        for(std::size_t i=0;i< Q.at(s).size();i++)
        {
            if(Visit.at(s).at(i) != 0)
            {
                max = Q.at(s).at(i);
            }
        }

        for(std::size_t i=0;i< Q.at(s).size();i++)
        {
            if(Visit.at(s).at(i) != 0)
            {
                if(Q.at(s).at(i)> max)
                {
                    max = Q.at(s).at(i);
                }
            }
        }
        V.at(s) = max;
        //V.at(s) = V.at(s) + alpha*(r+_discounting*V.at(sp)-V.at(s));
    }
    else
    {
        std::cout << "\n Desbordamiento de V(x) al intentar acceder a x = "<< s <<" o "<< sp <<"\n";
        std::cout << "Tamaño de V = "<< V.size()<<" \n";
    }

    return 0;
}

int LearningHandle::actionSelection(int s)
{
    //función que calcula la acción a realizar, según distribución de boltzmann
    int a=0; 
    
    //calculamos probabilidad de cada acción
    double sum =0;
    if(s < Q.size())
    {
        for(std::size_t i=0;i<Q.at(s).size();i++)
        {
            sum = sum + exp(Q.at(s).at(i)/temp);
        }
        for(std::size_t i=0;i<Q.at(s).size()-1;i++)
        {
            if(i < Prob.size())
                Prob.at(i)= exp(Q.at(s).at(i)/temp)/sum;
        }

        Prob.at(_numActions-1) = 1 - (Prob.at(0) + Prob.at(1) + Prob.at(2));
    }

    //seleccionamos con la probabilidad seleccionada con el método de monte carlo
    double u =(rand()% 100);
    u = u/100;
    double acumulativeProb =0;
    for(int i = 0; i < Prob.size();i++)
    {
        if( (acumulativeProb <= u) && (u < (acumulativeProb + Prob.at(i)) ) )
        {
            a=i;
        }
        acumulativeProb = acumulativeProb + Prob.at(i);
    }

    temp = temp *_lambda;
    return a;
}

int LearningHandle::actionSelectionE(int s)
{
    //función que calcula la acción a realizar, según e-greedy
    int a; 
    
    if((rand()%100) < (1-100*_epsilon))
    {
        //Explotación
        a = 0;
        for(std::size_t i = 0;i<Q.at(s).size();i++)
        {
            if(Q.at(s).at(i) > Q.at(s).at(a))
            {
                a = i;
            }
        }
    }
    else
    {
        //Exploración
        a = rand()%_numActions;
    }

    return a;
}

double LearningHandle::getReward(std::vector<int> state)
{
    //función que deveulve la recompensa, reward, según el estado
    double reward =0;
    if( (state.at(0)<1) )
    {
        reward = 100;
    }

    return reward;

}

bool LearningHandle::endOfLearningChecker()
{
    bool isCompleted = false;
    double max = 0;
    for (size_t i = 0; i < V.size(); i++)
    {
        if(abs(V.at(i)-PreviousV.at(i)) > max)
        {
            max = abs(V.at(i) - PreviousV.at(i));  
        }
    }
   
    double d = 10; //euclidean distance betwen v* and the desired V
    double e = d*(1-_discounting)/(2*_discounting);

    if(max < e)
    {
        isCompleted = true;
    }

    return isCompleted;
}

void LearningHandle::writeFiles(int episode, int s, int state, int action, double reward, bool endOfEpisode, std::vector<double> target, double neededTime)
{
    //función que escribe las tablas en archivos txt
    std::ostringstream ss;

    if ((s % 100) == 0)
    {
        
        //Tabla Q
        QFile << "\n"; 
        for(std::size_t i=0;i<Q.size();i++)
        {
            for(std::size_t j=0;j<Q.at(i).size();j++)
            {
                //comprobamos que no se scriben más de 10 caracteres
                ss.clear();
                ss << Q.at(i).at(j);
                if(ss.str().size() >= 10)
                {
                    QFile << limitNum(Q.at(i).at(j)) << " ";
                }
                else
                {
                    QFile << Q.at(i).at(j) << " ";
                }
            }

            QFile << "\n";   
        }
        QFile << "\n";
    }

    //Tabla V
    VFile << "\n"; 
    for(std::size_t j=0;j<V.size();j++)
    {
        ss.clear();
        ss << V.at(j);
        //comprobamos que no se scriben más de 10 caracteres
        if(ss.str().size() >= 10)
        {
            VFile << limitNum(V.at(j)) << " ";
        }
        else
        {
            VFile << V.at(j) << " ";
        }
        VFile << "\n"; 
    } 
    VFile << "\n";

    //Histórico, record
    RecordFile << state << " " << action << " " << reward << " "<< neededTime<<"\n";         
    
    //Fin de episodios
    if(endOfEpisode)
    {
        EpisodeFile << episode << " " << s << "\n";
        TargetFile << target.at(0) << " " << target.at(1) << " " << target.at(2) << "\n";
    }
}

void LearningHandle::closeFiles()
{
    std::ostringstream ss;

    //Tabla visit
    VisitFile << "\n"; 
    for(std::size_t i=0;i<Visit.size();i++)
    {
        for(std::size_t j=0;j<Visit.at(i).size();j++)
        {
            ss.clear();
            ss << Visit.at(i).at(j);
            //comprobamos que no se scriben más de 10 caracteres
            if(ss.str().size() >= 10)
            {
                VisitFile << limitNum(Visit.at(i).at(j)) << " ";
            }
            else
            {
                VisitFile << Visit.at(i).at(j) << " ";
            }
                  
        }

        VisitFile << "\n";   
    }
    VisitFile << "\n";

    //Tabla Q Final
    QFinal << "\n"; 
    for(std::size_t i=0;i<Q.size();i++)
    {
        for(std::size_t j=0;j<Q.at(i).size();j++)
        {
            ss.clear();
            ss <<Q.at(i).at(j);
            //comprobamos que no se scriben más de 10 caracteres
            if(ss.str().size() >= 10)
            {
                QFinal << limitNum(Q.at(i).at(j)) << " ";
            }
            else
            {
                QFinal << Q.at(i).at(j) << " ";
            }
            
        }

        QFinal << "\n";   
    }
    QFinal << "\n";



    QFile.close();
    QFinal.close();
    VisitFile.close();
    VFile.close();
    RecordFile.close();
    TargetFile.close();
    EndEpiInfo.close();
}

double LearningHandle::limitNum(double num)
{
    //método que limita el número de caracteres a 10
    std::ostringstream ss;
    double val; //valor que se devolverá
    int integer = round(num); //parte entera 
    ss << integer;
    int integerNum = ss.str().size(); //nuemro de caracteres de parte entera
    int decimalNum = 9-integerNum; //numero de caracteres que se van a permitir en parte decimal

    //si el númeor ya es mayor de 10 en parte entera, truncamos
    if(integerNum >= 10)
    {
        // si el bit más significativo que quedaría es 0, lo ponemos a 9
        if(ss.str().at(integerNum-10)=='0')
        {
            ss.str().at(integerNum-10)='9';
        }
        //numero truncado
        val = atof(ss.str().substr(integerNum-10,integerNum).c_str());
    }
    else
    {
        //redondeamos a parte decimal permitida
        val = round(num*pow(10,decimalNum))/pow(10,decimalNum);
    }
    return val;
}

void LearningHandle::upadateLastVisit(int s, int a)
{
    //método que actualiza la tabla visitas al final del episodio
    Visit.at(s).at(a)++;
}

void LearningHandle::saveEndEpiData(int episode, int sp, int ap)
{
    EndEpiInfo << episode << " " << sp << " " << ap << "\n";
}
#pragma endregion