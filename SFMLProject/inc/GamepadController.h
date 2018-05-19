#pragma once
#include <SFML\Window\Joystick.hpp>
#include <SFML\System\Vector2.hpp>
#include <iostream>
#include <math.h>

class GamepadController {
public:
    // Needs to have easily exposed buttons: not id's
    // 
    float triggerRawDeadzone = 20.f;
    float triggerPressScaledThreshold = .8f;
    float thumbstickRawDeadzone = 25.f;
    GamepadController() {
        
    }
    ~GamepadController() {

    }
    sf::Vector2f leftThumbstickValue() {
        //-100 to 100 converted to -1 to 1
        //std::cerr << "leftThumb: " << sf::Joystick::getAxisPosition(0, sf::Joystick::X)<< ", " << sf::Joystick::getAxisPosition(0, sf::Joystick::Y) << '\n';
        float x = sf::Joystick::getAxisPosition(0, sf::Joystick::X);
        if (x < thumbstickRawDeadzone && x > -thumbstickRawDeadzone)
            x = 0.0f;
        float y = sf::Joystick::getAxisPosition(0, sf::Joystick::Y);
        if (y < thumbstickRawDeadzone && y > -thumbstickRawDeadzone)
            y = 0.0f;
        return sf::Vector2f(x *.01f, y * .01f);
    }
    sf::Vector2f rightThumbstickValue() {
        //-100 to 100 converted to -1 to 1
        //std::cerr << "rightThumb: " << sf::Joystick::getAxisPosition(0, sf::Joystick::U)<< ", "<< sf::Joystick::getAxisPosition(0, sf::Joystick::R) << '\n';
        float x = sf::Joystick::getAxisPosition(0, sf::Joystick::U);
        if (x < thumbstickRawDeadzone && x > -thumbstickRawDeadzone)
            x = 0.0f;
        float y = sf::Joystick::getAxisPosition(0, sf::Joystick::R);
        if (y < thumbstickRawDeadzone && y > -thumbstickRawDeadzone)
            y = 0.0f;
        return sf::Vector2f(x * .01f,y * .01f);
    }
    float rightTriggerValue() {
        // 0 to 100
            //std::cerr << "leftTrig: "<< 10 << ": " <<  sf::Joystick::getAxisPosition(0,sf::Joystick::Axis::Z)<< '\n';
        float value = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::Z);
        if (value <= -triggerRawDeadzone) {
            return abs(value) * .01f;
        }
        return 0;
    }
    float leftTriggerValue() {
        //0 to -100 converted to 0 to 1
        //std::cerr << "rightTrig: " << sf::Joystick::getAxisPosition(0, sf::Joystick::Z) << '\n';
        float value = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::Z);
        if (value >= triggerRawDeadzone) {
            return value * .01f;
        }
        return 0;
    }
    bool pressedLeftTrigger() {
        return leftTriggerValue() >= triggerPressScaledThreshold;       
    }
    bool pressedRightTrigger() {
        return rightTriggerValue() >= triggerPressScaledThreshold;
    }
    bool pressedLeftBumper() {
        return sf::Joystick::isButtonPressed(0, 4);
    }
    bool pressedRightBumper() {
        return sf::Joystick::isButtonPressed(0, 5);
    }

    bool pressedA() {
        return sf::Joystick::isButtonPressed(0, 0);
    }
    bool pressedB() {
        return sf::Joystick::isButtonPressed(0, 1);
    }
    bool pressedX() {
        return sf::Joystick::isButtonPressed(0, 2);
    }
    bool pressedY() {
        return sf::Joystick::isButtonPressed(0, 3);
    }

    bool pressedBack() {
        return sf::Joystick::isButtonPressed(0, 6);
    }
    bool pressedStart() {
        return sf::Joystick::isButtonPressed(0, 7);
    }
    bool pressedLeftThumbstick() {
        return sf::Joystick::isButtonPressed(0, 8);
    }
    bool pressedRightThumbstick() {
        return sf::Joystick::isButtonPressed(0, 9);
    }

    bool pressedDpadUp() {
        return sf::Joystick::isButtonPressed(0, 0);
    }
    bool pressedDpadLeft() {
        return sf::Joystick::isButtonPressed(0, 0);
    }
    bool pressedDpadRight() {
        return sf::Joystick::isButtonPressed(0, 0);
    }
    bool pressedDpadDown() {
        return sf::Joystick::isButtonPressed(0, 0);
    }
};

/*
A Button joystick button 0

B Button joystick button 1

X Button joystick button 2

Y Button joystick button 3

L Button joystick button 4

R Button joystick button 5

Back joystick button 6

Start joystick button 7

Left Analog pressed joystick button 8

Right Analog pressed joystick button 9

*/