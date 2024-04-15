from brian2 import *
from Equations import *
from Params import *

""" Creates the neuron populations"""

def Create_Neurons(N_Prop_1,N_Prop_2, N_Tactile, N_GF, N_GF_M1, N_GF_M2):

    Prop_Neurons_1 = NeuronGroup(N_Prop_1, EQ_Prop_1, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    Prop_Neurons_1.v = c
    Prop_Neurons_1.segment_offset = 0 * ms

    Prop_Neurons_2 = NeuronGroup(N_Prop_2, EQ_Prop_2, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    Prop_Neurons_2.v = c
    Prop_Neurons_2.segment_offset = 0 * ms
  
    Prop_Neurons_T1 = NeuronGroup(N_Prop_1, EQ_Prop_T1, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')

    Prop_Neurons_T2 = NeuronGroup(N_Prop_2, EQ_Prop_T2, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')

    Tactile_Neurons = NeuronGroup(N_Tactile, EQ_Tactile, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    Tactile_Neurons.v = c
    Tactile_Neurons.segment_offset = 0 * ms

    GF_Neurons = NeuronGroup(N_GF, EQ_GF, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    GF_Neurons.v = c
    grid_size_x = N_Prop_1
    grid_size_y = N_Prop_2
    num_neurons = grid_size_x * grid_size_y
    # Generate x and y values for each neuron
    x_values = np.repeat(np.arange(grid_size_x), grid_size_y)
    y_values = np.tile(np.arange(grid_size_y), grid_size_x)

    # Assign x, y, and z values to each neuron
    GF_Neurons.X = x_values
    GF_Neurons.Y = y_values

    ###############

    MGF_Neurons_1 = NeuronGroup(N_GF_M1, EQ_MGF, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    MGF_Neurons_1.v = c
    grid_size_x = N_Prop_1
    grid_size_y = N_Prop_1
    num_neurons = grid_size_x * grid_size_y
    # Generate x and y values for each neuron
    x_values = np.repeat(np.arange(grid_size_x), grid_size_y)
    y_values = np.tile(np.arange(grid_size_y), grid_size_x)

    # Assign x, y, and z values to each neuron
    MGF_Neurons_1.X = x_values
    MGF_Neurons_1.Y = y_values

    ###############
    
    MGF_Neurons_2 = NeuronGroup(N_GF_M2, EQ_MGF, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')
    MGF_Neurons_2.v = c
    grid_size_x = N_Prop_2
    grid_size_y = N_Prop_2
    num_neurons = grid_size_x * grid_size_y
    # Generate x and y values for each neuron
    x_values = np.repeat(np.arange(grid_size_x), grid_size_y)
    y_values = np.tile(np.arange(grid_size_y), grid_size_x)
    # Assign x, y, and z values to each neuron
    MGF_Neurons_2.X = x_values
    MGF_Neurons_2.Y = y_values


    Directional_M1_Neurons = NeuronGroup(2, EQ_Prop_T1, threshold='v>Vt', reset='v = c;u += d', refractory=5 * ms, method='euler')


    Directional_M2_Neurons = NeuronGroup(2, EQ_Prop_T2, threshold='v>30*mV', reset='v = c;u += d', refractory=5 * ms, method='euler')
    # Directional_M2_Neurons=0

    return Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, Tactile_Neurons, GF_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons