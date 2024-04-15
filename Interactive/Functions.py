import numpy as np
import matplotlib.pyplot as plt
from brian2 import *



#   Approximate RGB values based on given Min and Max Values 
def rgb(minimum, maximum, value):
    minimum, maximum = float(minimum), float(maximum)
    ratio = 2 * (value-minimum) / (maximum - minimum)
    b = int(max(0, 255*(1 - ratio)))
    r = int(max(0, 255*(ratio - 1)))
    g = 255 - b - r
    return (r, g, b)


#   Visualize Synapse coonectivity (Pre Post Neurons)

def visualise_connectivity(S):
    Ns = len(S.source)
    Nt = len(S.target)
    figure(figsize=(30, 30))
    subplot(121)
    plot(zeros(Ns), arange(Ns), 'ok', ms=10)
    plot(ones(Nt), arange(Nt), 'ok', ms=10)
    for i, j in zip(S.i, S.j):
        plot([0, 1], [i, j], '-k')
    xticks([0, 1], ['Source', 'Target'])
    ylabel('Neuron index')
    xlim(-0.1, 1.1)
    ylim(-1, max(Ns, Nt))
    subplot(122)
    plot(S.i, S.j, 'ok')
    xlim(-1, Ns)
    ylim(-1, Nt)
    xlabel('Source neuron index')
    ylabel('Target neuron index')
    plt.show()


#   Visualize Synapse coonectivity (Post and Weight )

def visualise_connectivity2(S):
    Ns = len(S.source)
    Nt = len(S.target)
    # figure(figsize=(30, 30))
    
    # # Subplot 1: Neuron index
    # subplot(121)
    # plot(zeros(Ns), arange(Ns), 'ok', ms=10)
    # plot(ones(Nt), arange(Nt), 'ok', ms=10)
    # for i, j, weight in zip(S.i, S.j, S.w/mV*ms):
    #     plot([0, 1], [i, j], '-k', linewidth=weight)  # Use linewidth to represent weight
    # xticks([0, 1], ['Source', 'Target'])
    # ylabel('Neuron index')
    # xlim(-0.1, 1.1)
    # ylim(-1, max(Ns, Nt))

    # Subplot 2: Scatter plot with size based on weight
    # subplot(122)
    scatter(S.i, S.j, s=np.abs(S.w) * .1, c='r', marker='o')  # Adjust marker size based on weight
    xlim(-1, Ns)
    ylim(-1, Nt)
    xlabel('Source neuron index')
    ylabel('Target neuron index')
    
    plt.show()
