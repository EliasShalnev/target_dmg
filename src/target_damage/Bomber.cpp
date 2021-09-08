#include "target_damage/Bomber.h"

#include <cmath>

#include "target_damage/Observer.h"


Bomber::Bomber(const ModelName& modelName, const BomberName& bomberName, Observer& observer)
    : Model(modelName)
    , m_bomberName(bomberName)
    , m_observer(observer)
    , m_uavState(m_nh, "/" + bomberName + "/mavros/state", 10)
    , m_dropPoint( m_nh.subscribe<geometry_msgs::Point>
                   ("/" + bomberName + "/drop_point", 10, &Bomber::dropPointCallback, this) )
{ }


bool Bomber::isActive() const
{
    return m_uavState.getMessage()->armed;
}


geometry_msgs::Point::ConstPtr Bomber::evalHitPoint(const geometry_msgs::Point::ConstPtr& dropPoint) 
{
    //Bomb parameters
    const double kk = 0.4; //коэффициент формы(шар) из книги
    const double m = 0.5; //масса спец груза
    const auto s = 3.14*std::pow(0.05, 2); //площадь груза

    //Environment parameters
    const double g = 9.8;
    const uint32_t po = 101325; //атмосферное давление
    //TODO - брать откуда-то
    const double T0 = 293.15; //температура на уровне моря. (кельвины) 
    const double L = 0.0065; //вертикальный градиент температуры
    const double R = 8.31446; //универсальная газовая постоянная
    const double MM = 0.0289652; //молярная масса сухого воздуха

    //TODO - hardcode get wind speed from gazebo plugin
    const double windSpeed_x = 0; //скорость ветра по x
    const double windSpeed_y = 0; //скорость ветра по y

    //Bomber parameters
    auto height = getCoordinates()->z;
    const auto uavSpeed_x = getMovementSpeed()->x; //скорость самолета по оси х
    const auto uavSpeed_y = getMovementSpeed()->y; //скорость самолета по оси y
    const auto uavSpeed_z = getMovementSpeed()->z; //скорость самолета по оси z

    ROS_INFO_STREAM("Bomber \"" << m_bomberName << "\" parameters:");
    ROS_INFO_STREAM("- height: " << height);
    ROS_INFO_STREAM("- linear speed x: " << uavSpeed_x);
    ROS_INFO_STREAM("- linear speed y: " << uavSpeed_y);
    ROS_INFO_STREAM("- linear speed z: " << uavSpeed_z);

    // const auto time = std::sqrt(2*height/g); //приближенное время падения

    auto bombSpeed_x = uavSpeed_x; //скорость СГ по оси x
    auto bombSpeed_y = uavSpeed_y; //скорость СГ по оси y
    auto bombSpeed_z = uavSpeed_z; //скорость СГ по оси z

    const float dt = 0.1;
    double dx = 0.0;
    double dy = 0.0;

    double fallTime = 0; //время падения

    while(height > 0)
    {
        const double temp = std::pow(1-(L*height)/T0, (g*MM)/(R*L)-1);
        const double ro = (po*MM*temp)/(R*T0); //плотность воздуха

        auto diff = windSpeed_x - uavSpeed_x;
        auto sign = std::signbit(diff) ? -1.0 : 1.0;
        const auto bombAccel_x = ( kk*s*ro*sign*std::pow(diff, 2) )/(2*m); //ускорение СГ c учетом воздушной среды по оси x

        diff = windSpeed_y - uavSpeed_y;
        sign = std::signbit(diff) ? -1.0 : 1.0;
        const auto bombAccel_y = ( kk*s*ro*sign*std::pow(diff, 2) )/(2*m); //ускорение СГ с учетом воздушной среды по оси y

        bombSpeed_x = bombSpeed_x + bombAccel_x*dt;
        bombSpeed_y = bombSpeed_y + bombAccel_y*dt;

        dx = dx + bombSpeed_x*dt;
        dy = dy + bombSpeed_y*dt;

        const auto air_forse = ( kk*s*ro*std::pow(bombSpeed_z, 2) )/(2*m); //сила сопротивления воздуха
        bombSpeed_z = bombSpeed_z + (g - air_forse)*dt;
        height = height - bombSpeed_z*dt;
        fallTime = fallTime + dt;
    }

    geometry_msgs::Point::Ptr hitPoint(new geometry_msgs::Point);
    
    hitPoint->x = dropPoint->x + dx;
    hitPoint->y = dropPoint->y + dy;

    ROS_INFO_STREAM("Bomb parameters:");
    ROS_INFO_STREAM("- Drop point:" << " x=" << dropPoint->x 
                                    << " y=" << dropPoint->y);
    ROS_INFO_STREAM("- Fall time: " << fallTime);
    ROS_INFO_STREAM("- Hit point: x=" << hitPoint->x << " y=" << hitPoint->y);

    return hitPoint;
}


void Bomber::dropPointCallback(const geometry_msgs::Point::ConstPtr& dropPoint) 
{
    //TODO - uncomment
    // if( !isActive() ) { return; }
  
    auto hitPoint = evalHitPoint(dropPoint);
    m_observer.evalDamage(hitPoint);
}
