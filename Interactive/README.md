🤖🎮 **Interactive Model of Self-Reaching**

Run `Interactive_1.py` to activate the simulation. Clicking on the torso activates tactile receptive fields, visible on the middle-left. The model then attempts to reach the activated point using its trained network. Current and target joint angle neural coding is displayed on the lower-left. Current touch activation is shown on the top-left, and Gain-Field neuron activation is plotted at the bottom of the figure.

## `Interactive_1.py` - 2D Serial Arm Simulation

This script simulates a 2D serial arm using Pygame, NumPy, and other libraries. It models a robotic arm with two joints controlled by proprioceptive and tactile sensors. The simulation involves running a spiking neural network (SNN) to control the arm's movement based on sensor inputs.

### Initialization and Constants

- **Pygame Initialization:** Initializes Pygame for visualization.
- **Visualization Constants:** Defines constants for screen dimensions, arm lengths, sensor grid size, etc.

### Network Parameters

- Defines parameters such as the number of proprioception neurons, tactile neurons, simulation duration, etc.
- Creates neuron populations, synapses, and monitors using helper functions.

### Input Data Preparation

- Prepares input data arrays for proprioceptive and tactile sensors.
- Creates timed arrays for input data.

### Main Simulation Loop

- Iterates over simulation steps, updating the arm's position and sensor activations.
- Runs the SNN for each iteration to compute neural activities.
- Calculates motor neuron activity and updates joint angles.
- Calculates proprioceptive and tactile neuron activity.
- Visualizes the arm, sensor activations, and neural activity using Pygame.

### Visualization

- Draws the arm, including the body, forearm, hand, and finger.
- Plots bar charts representing current and target activations of proprioceptive neurons.
- Displays heatmaps representing tactile sensor activations and gain field neuron activations.
- Visualizes touch sensors on the body square.
- Allows interaction with the simulation by clicking on the screen.

### Data Storage (Optional)

- Optionally stores simulation data in a CSV file for further analysis.

This README provides an overview of the simulation script, including its components, functionalities, and visualization outputs. It serves as a guide for understanding and using the simulation for studying arm control in a spiking neural network context.

📊 **Weights Histogram**

This script visualizes the connectivity of neural networks using histograms and scatter plots. It utilizes Python libraries such as NumPy, Matplotlib, and custom functions to create and visualize neuron populations, synapses, and monitors.

### Importing Libraries and Modules

- **numpy:** Importing NumPy library for numerical computations.
- **Functions:** Custom functions for visualization and data processing.
- **Create_Neurons:** Module for creating neuron populations.
- **Create_Synapses:** Module for creating synapses between neuron populations.
- **Create_Monitors:** Module for creating monitors to track neural activity.
- **Plot_Results:** Module for plotting simulation results.
- **matplotlib.pyplot:** Importing Matplotlib library for plotting.
- **mpl_toolkits.mplot3d.Axes3D:** Importing Axes3D from Matplotlib for 3D plotting.
- **matplotlib.animation.FuncAnimation:** Importing FuncAnimation for animation.

### Network Parameters

- Defines parameters such as the number of proprioception neurons, tactile neurons, and gain field neurons.
- Specifies the duration of each simulation run.

### Creating Neurons and Synapses

- Calls functions from the `Create_Neurons` and `Create_Synapses` modules to create neuron populations and synapses.
- Initializes neuron populations and synapses based on specified parameters.
- Loads pre-trained synapse weights from files.

### Visualization

- Loads pre-trained synapse weights and plots their distribution using histograms.
- Visualizes the connectivity of synapses between neuron populations using scatter plots.
- Calls the custom function `visualise_connectivity2` to visualize synapse connectivity between gain field neurons and tactile neurons.
- Calls the same function to visualize synapse connectivity between tactile neurons and gain field neurons.

This script provides tools for visualizing the connectivity of neural networks, including histograms of synapse weights and scatter plots of synapse connections. It helps researchers and practitioners understand the structure and dynamics of neural networks, aiding in the analysis and interpretation of simulation results.

🧠 **Functions**

This script contains several visualization functions implemented using NumPy, Matplotlib, and Brian2 libraries. These functions are designed to visualize the connectivity and properties of synapses in a spiking neural network simulation.

### Function: rgb

This function approximates RGB values based on given minimum and maximum values and a target value. It returns a tuple representing the RGB color corresponding to the target value within the specified range.

### Function: visualise_connectivity

This function visualizes the connectivity between pre and post neurons in a synapse object (`S`). It plots two subplots: one showing the connectivity pattern between the source and target neurons, and the other showing the neuron indices of source and target neurons.

### Function: visualise_connectivity2

This function visualizes the connectivity between post neurons and the corresponding weight of synapses. It plots a scatter plot where the size of the markers represents the absolute weight of the synapse.

These visualization functions provide insights into the connectivity patterns and properties of synapses in spiking neural networks, aiding in the analysis and understanding of network behavior.

🧠 **Create_Monitors.py**

This Python script contains functions for creating spike monitors and neuron state monitors using the Brian2 library, which is a simulator for spiking neural networks. The monitors are essential tools for observing and recording the activity of neurons and synapses during simulation.

### Spike Monitors

#### Function: Create_SP_Monitors

This function creates spike monitors for different types of neurons in the network.

### Neuron State Monitors

#### Function: Create_Neuron_ST_Monitors

This function creates state monitors to record the membrane potential of neurons and additional variables if needed.

### Synapse State Monitors

#### Function: Create_Synapse_ST_Monitors

This function creates state monitors to observe the state of synapses.

### Test Spike Monitors

#### Function: Create_Test_SP_Monitors

This function creates spike monitors for testing purposes, such as monitoring activity in vision neurons and retinal-tactile neurons.

These functions facilitate the observation and analysis of neuronal activity and synaptic dynamics during simulation, aiding in the development and understanding of spiking neural networks.

🧠 **Create_Neurons.py**

This Python script contains a function to create different neuron populations using the Brian2 library. Neuron populations are essential components of spiking neural network simulations.

### Function: Create_Neurons

This function initializes several neuron populations based on predefined equations and parameters.

🔗 **Create_Synapses.py**

This script contains a function to create and load synapses between different neuron populations using the Brian2 library. Synapses play a crucial role in transmitting signals between neurons in a spiking neural network.

### Function: Load_Synapses

This function creates synapses between various neuron populations and loads pre-trained synaptic weights.
