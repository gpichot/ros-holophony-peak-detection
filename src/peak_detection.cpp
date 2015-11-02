#include "ros/ros.h"
#include "audiocorr/audiocorr_result.h"
#include "peak_detection/SoundSources.h"
#include <cmath>
#include <cassert>
#define V_SOUND 0.34 // m/ms
#define D_MIC 0.2 // Distance entre 2 microphones

struct Direction {
  float x;
  float theta;
};

struct Point {
  float x;
  float y;
};


Point calcPosition(Direction& a, Direction& b){
  Point point;
  float cota = 1. / tan(a.theta);
  float cotb = 1. / tan(b.theta);
  if(b.theta != a.theta){

    point.x = (a.x * cota - b.x * cotb) / (cota - cotb);
    point.y = cota * (point.x - a.x);
    return point;
  }
  else{
    point.x = 1000;
    point.y = 1000;
  }
}
Direction calcDirection(float delay, int mic1, int mic2) {

  /* 
     La direction du son est donnée par la branche d'une hyperbole.
     On repère cette direction par la position du milieu des 2 micros (x)
     et par l'angle entre l'asymptote à la branche d'hyperbole et l'axe des micros.

     Equations de l'hyperbole :

     |MF - MF'| = 2a = delay * V_SOUND
asymptote : y = +- b/a * x
avec b = sqrt(c^2 - a^2)
   */

  float a = fabs(V_SOUND * delay / 2.);
  float c = D_MIC / 2.;

  std::cout << a << std::endl;
  Direction direction;
  direction.x = mic1 * D_MIC + c * (mic2 - mic1);
  float b = pow(c,2) - pow(a,2);

  direction.theta = (delay < 0 ? -1 : 1) * atan(a / sqrt(fabs(b)));

  return direction;
}

class SubscribeAndPublish
{
  public:
    SubscribeAndPublish()
    {

      _sub = _n.subscribe("audiocorr_result", 2, &SubscribeAndPublish::resultCallback, this);

      _pub = _n.advertise<peak_detection::SoundSources>("sound_sources", 3);

    }

    /* Cette fonction est appelée à chaque réception de message */
    void resultCallback(const audiocorr::audiocorr_result message)
    {

      int i, j;
      const int nb_mics = message.mic1.size();
      const int nb_pts = message.data.size() / nb_mics;
      const int window_size = nb_pts / 5;
      const int origin = nb_pts / 2;
      const int window_begin = nb_pts / 2 - origin; 
      const int window_end = nb_pts / 2 + origin; 

      Direction directions[nb_mics];
      peak_detection::SoundSources msg;

      for(j = 0; j < nb_mics; j++) {
        float max = 0;
        int offset = origin;
        for(i = window_begin; i <= window_end; i++) {
          if(message.data[nb_mics * i + j] > max) {
            max = message.data[nb_mics * i + j];
            offset = i;
          }
        }
        float delay = float(origin - offset) / message.fe * 1000; // Conversion du délai en ms.

        std::cout << "Mic " << message.mic1[j] << "-" << message.mic2[j] << ": " << delay << "ms" << std::endl;

        directions[j] =  calcDirection(delay, message.mic1[j], message.mic2[j]);


        /*
           std::cout << "x : " << direction.x << std::endl;
           std::cout << "theta : " << 180 / 3.1416 * direction.theta << std::endl;
         */
      }
      int nb_intersections = nb_mics * (nb_mics - 1) / 2;
      Point intersections[nb_mics][nb_mics - 1];

      for(int i = 0 ; i < nb_mics ; i++){
        for(int j = i + 1; j < nb_mics; j++) {
          intersections[i][j] = calcPosition(
              directions[i],
              directions[j]
              );
          peak_detection::SourcePosition pos;
          pos.x = intersections[i][j].x;
          pos.y = intersections[i][j].y;
          msg.intersections.push_back(pos);
        
          std::cout << "x : " << intersections[i][j].x << std::endl;
          std::cout << "y : " << intersections[i][j].y << std::endl;
        }
      }


      _pub.publish(msg);
    }

  private:
    ros::NodeHandle _n; 
    ros::Publisher _pub;
    ros::Subscriber _sub;

};







int main(int argc, char **argv)
{
  ros::init(argc, argv, "peak_detection");

  SubscribeAndPublish sap;

  ros::spin();

  return 0;
}
