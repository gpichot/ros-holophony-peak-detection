#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include "peak_detection/SoundSources.h"


/** 
 * Number of microphones
 */
static const int NB_MICS = 8;

/**
 * Scale factor.
 */
static const int SCALE_FACTOR = 200;

/**
 * Distance between microphones in meters * SCALE_FACTOR.
 */
static const float MICS_DISTANCE = 0.20;

/**
 * Distance of the microphone bar to the wall in meters * SCALE_FACTOR.
 */
static const float WALL_DISTANCE = 0.20;


class SmartRoom {
    public:
        inline int scaleX(float x) const {
            return 30 + x * SCALE_FACTOR;
        }
        inline int scaleY(float y) const {
            return 30 + y * SCALE_FACTOR;
        }
        SmartRoom(sf::RenderWindow * window) : _window(window) {
            _subscriber = _node.subscribe("sound_sources", 4, &SmartRoom::redraw, this);

        }
        /**
         * Redraw when a new message arrives from ros.
         */
        void redraw(const peak_detection::SoundSources sources) {
            _window->clear();

            drawRoom();
            
            // Draw Intersections
            const int nb_positions = sources.intersections.size();
            for(int i = 0; i < nb_positions; i++) {
                peak_detection::SourcePosition position = sources.intersections.at(i);
                std::cout << "(x: " << position.x << ", y: " << position.y << ")" << std::endl; 
                sf::CircleShape circle(5);
                circle.setPosition(scaleX(position.x), scaleY(position.y));
                _window->draw(circle);
            }

            // Draw Directions
            const int nb_directions = sources.directions.size();
            sf::VertexArray intersections(sf::Lines, nb_directions * 2);
            for(int i = 0; i < nb_directions; i++) {
                peak_detection::SourceDirection direction = sources.directions.at(i);
                double x0, y0, x1, y1;
                if(direction.theta == 0) {
                    x0 = direction.x;
                    y0 = 0.;
                    x1 = direction.x;
                    y1 = 300.;
                } else {
                    x0 = direction.x;
                    x1 = 0.;
                    y0 = 0.;
                    y1 = direction.x / tan( - direction.theta);
                }
                intersections[2 * i].position = sf::Vector2f(scaleX(x0), scaleY(y0));
                intersections[2 * i + 1].position = sf::Vector2f(scaleX(x1), scaleY(y1));
                intersections[2 * i].color = sf::Color::White;
                intersections[2 * i + 1].color = sf::Color::White;
            }
            _window->draw(intersections);

            _window->display();
        }
    private:
        ros::NodeHandle _node;
        ros::Subscriber _subscriber;
        sf::RenderWindow * _window;

        /**
         * It draws the room static objects.
         *
         * It draws microphones and walls
         */
        void drawRoom() {
            // Microphones
            // TODO save the microphones (they will never move)
            sf::VertexArray microphones(sf::Points, NB_MICS);
            for(int i = 0; i < NB_MICS; i++) {
                microphones[i].position = sf::Vector2f(
                    scaleX((NB_MICS - 1 - i) * MICS_DISTANCE),
                    scaleY(WALL_DISTANCE)
                );
                microphones[i].color = sf::Color::White;
            }
            _window->draw(microphones);
        }

};

int main(int argc, char **argv)
{
    sf::RenderWindow window(sf::VideoMode(600, 480), "SFML works!");
    ros::init(argc, argv, "smart_room_plot");

    SmartRoom smartroom(&window);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        ros::spin();
    }

    return 0;
}
