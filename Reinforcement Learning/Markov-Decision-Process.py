#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np

class AuscultationMDP:
    def __init__(self, signal_quality, signal_length, discount_factor=0.9):
        self.signal_quality = signal_quality
        self.signal_length = signal_length
        self.discount_factor = discount_factor
        self.state_space = [(i, j) for i in range(signal_quality + 1) for j in range(signal_length + 1)]
        self.action_space = ["Filter", "FeatureExtraction", "Classification"]
        self.transitions = self.build_transitions()

    def build_transitions(self):
        transitions = {}
        for state in self.state_space:
            transitions[state] = {}
            for action in self.action_space:
                transitions[state][action] = self.calculate_transitions(state, action)
        return transitions

    def calculate_transitions(self, state, action):
        current_quality, current_length = state

        # Transition dynamics based on actions
        if action == "Filter":
            next_quality = min(current_quality + 1, self.signal_quality)
            next_length = current_length
        elif action == "FeatureExtraction":
            next_quality = current_quality
            next_length = min(current_length + 1, self.signal_length)
        elif action == "Classification":
            next_quality = max(current_quality - 1, 0)
            next_length = min(current_length + 1, self.signal_length)

        # Rewards based on the resulting state
        reward = self.calculate_reward(next_quality, next_length)

        return [(1.0, (next_quality, next_length), reward)]

    def calculate_reward(self, quality, length):
        # Example: Reward function based on signal quality and processing length
        return quality - length

def value_iteration(mdp, theta=0.0001):
    V = {state: 0 for state in mdp.state_space}
    
    while True:
        delta = 0
        for state in mdp.state_space:
            v = V[state]
            max_value = float("-inf")
            for action in mdp.action_space:
                action_value = sum(prob * (reward + mdp.discount_factor * V[next_state])
                                   for prob, next_state, reward in mdp.transitions[state][action])
                max_value = max(max_value, action_value)
            V[state] = max_value
            delta = max(delta, abs(v - V[state]))
        if delta < theta:
            break
    
    policy = {state: get_best_action(mdp, state, V) for state in mdp.state_space}
    return V, policy

def get_best_action(mdp, state, V):
    if state[1] == mdp.signal_length:
        return None  # Terminal state reached

    best_action = None
    max_value = float("-inf")

    for action in mdp.action_space:
        action_value = sum(prob * (reward + mdp.discount_factor * V[next_state])
                           for prob, next_state, reward in mdp.transitions[state][action])
        if action_value > max_value:
            max_value = action_value
            best_action = action

    return best_action

def main():
    # Auscultation MDP
    signal_quality = 3
    signal_length = 5
    auscultation_mdp = AuscultationMDP(signal_quality, signal_length)

    # Value iteration
    optimal_values, optimal_policy = value_iteration(auscultation_mdp)

    print("Optimal Values:")
    print(optimal_values)
    print("\nOptimal Policy:")
    print(optimal_policy)

if __name__ == "__main__":
    main()

