import numpy as np
import Functions as FN
from Create_Neurons import *
from Create_Synapses import *
from Create_Monitors import *
from Plot_Results import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


#   Define Network Parameters
N_Prop_1 = 10
N_Prop_2 = 10
N_Tactile = 9
N_GF = N_Prop_1 * N_Prop_2


N_Prop_1 = 10
N_Prop_2 = 10
N_Tactile = 9
N_GF = N_Prop_1 * N_Prop_2
N_GF_M1 = N_Prop_1 * N_Prop_1
N_GF_M2 = N_Prop_2 * N_Prop_2
Run_Duration = 100 * ms

Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, Tactile_Neurons, GF_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons = Create_Neurons(N_Prop_1,N_Prop_2, N_Tactile, N_GF, N_GF_M1, N_GF_M2)
Prop1_GF, Prop2_GF,GF_Prop1, GF_Prop2, GF_Tactile, Tactile_GF, Prop1_MGF, Prop2_MGF, TProp1_MGF, TProp2_MGF, MGF1_DM1, MGF2_DM2, GF_GF_Inh, GF_GF_Ex = Load_Synapses(Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, GF_Neurons, Tactile_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons)

SynapseDir = 'C:/Users/pabdollahzadeh/Desktop/Codes/Tactile_Prop/v24_11/Training_Params/'
Syn_Name = 'GF_Tactile'
Syn_Suff = '_TR1_24_11_epoch400'
W = np.load(SynapseDir+Syn_Name+Syn_Suff+'.npy')
plt.hist(W, 100)
plt.show()
# plt.hist(GF_Tactile.w, 100)
# plt.show()

FN.visualise_connectivity2(GF_Tactile)
FN.visualise_connectivity2(Tactile_GF)