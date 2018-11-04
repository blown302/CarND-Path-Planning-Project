//
// Created by Thomas Milas on 11/1/18.
//

#ifndef PATH_PLANNING_STATE_MACHINE_H
#define PATH_PLANNING_STATE_MACHINE_H

#include <unordered_map>
#include <functional>

template <class T>
class StateMachine {
private:
    T state;
//    std::unordered_map<T, void*()> actions;
public:
    StateMachine(T state): state(state) {};
    void process();
    void registerAction(T state, std::function<void()> func)  {
//        actions[state] = func;
    };

};

#endif //PATH_PLANNING_STATE_MACHINE_H
