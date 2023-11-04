#pragma once

#include <chrono>
#include <Windows.h>
#include <iostream>


class KeyController {
public:
    KeyController(char key, bool enable_logging = true) : key(key), enable_logging(enable_logging){
        last_call = std::chrono::high_resolution_clock::now();
    };
    KeyController() {};

    void press(float minimal_delay = 0);
    void release();
    ~KeyController();

private:
    bool is_pressed = false;
    bool enable_logging;
    std::chrono::steady_clock::time_point last_call;
    char key = '\0';
};

class Steer {
  public:
    Steer(char forward_key, char backward_key, char left_key, char right_key, bool logging);
    Steer() = delete;
    Steer(const Steer&) = delete;

    void left(float angle);
    void right(float angle);
    void forward();
    void backward();
    void goStraight();
    void release();
    ~Steer();

  private:
    float getDelay(float angle);

    KeyController controllers[4];
    KeyController* forward_ = controllers;
    KeyController* backward_ = controllers + 1;
    KeyController* left_ = controllers + 2;
    KeyController* right_ = controllers + 3;
    float max_delay_ = 300;
    float max_angle_ = 60;
    
};