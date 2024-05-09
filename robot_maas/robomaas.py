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
        """
        o_pre (o_size,1)
        o_next (o_size,1)
        """

        prediction_error = self.Q@o_next-(self.Q@o_pre+self.V[:,action])
        return prediction_error

    # def plan(self, start, goal, env):
    #     a_record = []
    #     o_record = []
    #     loc = start
    #     for i in range(self.o_size):
    #         o_record.append(loc)
    #         if loc==goal:
    #             return i, o_record
    #         loc, action = self.move_one_step(loc, goal, a_record,)
    #         a_record.append(action)

    #     return i, o_record

    def move_one_step(self, loc, goal, a_record, affordance):#TODO: delte last arg
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
        return action_idx # TODO: chenge where method is called
    
if __name__ == '__main__':

    norm=False
    model = Agent(o_size=2, a_size=8, s_dim=1000)
    device = 'cpu'
    loss_record = []

    plt.figure()

    while True:
        # training step
        with torch.no_grad():
            o_pre = torch.tensor([14., 72.])
            action = 0
            o_next = torch.tensor([14., 72.])

            identity = torch.eye(model.a_size).to(device)
            state_diff = model.Q@o_next-model.Q@o_pre
            prediction_error = state_diff - model.V[:,action]
            desired = identity[action].T # TODO: maybe remove?

            # Core learning rules:
            print(model.Q.shape, prediction_error.shape, o_next.shape, torch.outer(prediction_error, o_next).shape)
            model.Q += -0.1 * torch.outer(prediction_error, o_next)#TODO:o.T?
            model.V[:,action] += 0.01 * prediction_error
            if norm:
                model.V.data = model.V / torch.norm(model.V, dim=0)

            loss = nn.MSELoss()(prediction_error, torch.zeros_like(prediction_error))
            loss_record.append(loss.cpu().item())

            plt.plot(loss_record)
            plt.pause(0.001)
            plt.draw()
