import matplotlib.pyplot as plt

import torch
import torch.nn as nn


class Agent(nn.Module):
    def __init__(self, o_size, a_size, s_dim, device='cpu'):
        super(Agent, self).__init__()
        self.Q = nn.Parameter(1*torch.randn(s_dim, o_size, device=device))
        self.V = nn.Parameter(0.1*torch.randn(s_dim, a_size, device=device))
        self.o_size = o_size
        self.a_size = a_size

        self.device = device

    def forward(self, o_pre, action, o_next):
        prediction_error = self.Q@o_next-(self.Q@o_pre+self.V[:,action])
        return prediction_error

    def move_one_step(self, loc, goal, a_record, affordance):
        """affordance = allowed actions (list)
        a_record has to be empty
        """
        
        affordance_vector = torch.ones(self.a_size, device=self.device) * (-1e6)
        affordance_vector[affordance] = 0
        affordance_vector_fix = affordance_vector.clone()
        not_recommended_actions = a_record
        affordance_vector_fix[not_recommended_actions] *= 0.

        delta = self.Q@goal-self.Q@loc

        utility = (self.V.T@delta) + affordance_vector

        action_idx = torch.argmax(utility).item()

        print("affordance_vector",affordance_vector.detach().numpy(),"utility", utility.detach().numpy(), "action_udx", action_idx)
        return action_idx
    
    def move_one_step_8(self, loc, goal, a_record, affordance):
        affordance_vector = torch.ones(2*self.a_size, device=self.device) * (-1e6)
        affordance_vector[affordance] = 0
        affordance_vector_fix = affordance_vector.clone()
        not_recommended_actions = a_record
        affordance_vector_fix[not_recommended_actions] *= 0.

        delta = self.Q@goal-self.Q@loc

        V_diag = self.V + self.V.roll(1, 1)
        utility = ((torch.cat([self.V, V_diag], dim=1)).T@delta) + affordance_vector

        action_idx = torch.argmax(utility).item()

        print("affordance_vector",affordance_vector.detach().numpy(),"utility", utility.detach().numpy(), "action_udx", action_idx)
        return action_idx
    

# TODO: remove!
ACTION_SPACE = torch.tensor(ACTION_SPACE)
ACTION_SPACE_DIAG = ACTION_SPACE + ACTION_SPACE.roll(1, 1)
ACTION_SPACE = torch.cat([ACTION_SPACE, ACTION_SPACE_DIAG], dim=1)