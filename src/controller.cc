#include "controller.h"
#include <cmath>
#include <algorithm>

void KeyController::press(float minimal_delay) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> delay = now - last_call;
        if (delay.count() > minimal_delay) {
            if (enable_logging) {
                std::cout << key<< " is pressed" << std::endl;
            }
            INPUT input;
            input.type = INPUT_KEYBOARD;
            input.ki.wVk = key;
            input.ki.dwFlags  = KEYEVENTF_EXTENDEDKEY;
            UINT uSent = SendInput(1, &input, sizeof(INPUT));
            is_pressed = true;
            last_call = now;
        }
    }
    void KeyController::release() {
        if (is_pressed) {
            INPUT input;
            input.type = INPUT_KEYBOARD;
            input.ki.wVk = key;
            input.ki.dwFlags = KEYEVENTF_KEYUP;
            UINT uSent = SendInput(1, &input, sizeof(INPUT));
            is_pressed = false;
        }
    }

    KeyController::~KeyController() {
        release();
    }

    Steer::Steer(char forward_key, char backward_key, char left_key, char right_key, bool logging) {
        *forward_ = KeyController(forward_key, logging);
        *backward_ = KeyController(backward_key, logging);
        *left_ = KeyController(left_key, logging);
        *right_ = KeyController(right_key, logging);
    }

    void Steer::left(float angle)
    {
        right_->release();
        left_->release();
        left_->press(getDelay(angle));
    }

    void Steer::right(float angle)
    {
        left_->release();
        right_->release();
        right_->press(getDelay(angle));
    }

    void Steer::forward()
    {
        backward_->release();
        forward_->press();
    }

    void Steer::backward()
    {
        forward_->release();
        backward_->press();
    }
    void Steer::release()
    {
        for (int i = 0; i < 4; i++) {
            controllers[i].release();
        }
    }

    float Steer::getDelay(float angle)
    {
        return max_delay_ * std::clamp((max_angle_ - fabs(angle))/ max_angle_, 0.1f, 1.f);
    }

    void Steer::goStraight() {
        left_->release();
        right_->release();
    }

    Steer::~Steer()
    {
        release();
    }
