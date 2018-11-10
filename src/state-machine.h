//
// Created by Thomas Milas on 11/1/18.
//

#ifndef PATH_PLANNING_STATE_MACHINE_H
#define PATH_PLANNING_STATE_MACHINE_H

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <vector>
#include <exception>
#include <iostream>

template <class T, class TR>
class StateMachine {
private:
    T state;
    std::unordered_map<T, std::function<void()>, std::hash<int>> m_entry_transitions;
    std::unordered_map<T, std::function<void()>, std::hash<int>> m_actions;
    std::unordered_map<T, std::unordered_map<TR, T, std::hash<int>>, std::hash<int>> m_allowed_transitions;
    bool isAllowed(TR trigger) {
        return m_allowed_transitions[state].find(trigger) != m_allowed_transitions[state].end();
    }
public:
    StateMachine(T state): state(state) {};
    void process(){
        m_actions[state]();
    };
    void registerAction(T state, const std::function<void()> &func)  {
        m_actions[state] = func;
    };
    void onEntry(T state, std::function<void()> func) {
        m_entry_transitions[state] = func;
    }
    void allow(T from_state, T to_state, TR trigger) {
        m_allowed_transitions[from_state][trigger] = to_state;
    };
    void fire(TR trigger) {
        if (!isAllowed(trigger)) {
            throw "state transition is not allowed";
        }
        state = m_allowed_transitions[state][trigger];
        // run on entry
        if (m_entry_transitions.find(state) != m_entry_transitions.end())
            m_entry_transitions[state]();
        std::cout << "changing state to " << state << std::endl;
    };
};

#endif //PATH_PLANNING_STATE_MACHINE_H
