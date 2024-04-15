from brian2 import *

a = 0.02 / ms                       # Izhikivich Neuron Model's Param
b = 0.2 / ms                        # Izhikivich Neuron Model's Param
c = -65 * mV                        # Izhikivich Neuron Model's Param
d = 8 * mV / ms                     # Izhikivich Neuron Model's Param

tau = 10 * ms                       # time constant
Vr = 0 * mV                         # reset potential
Vt = 30 * mV                        # spike threshold
taugd =  5 * ms                     # GABA Decay Time constant
tauad =  2 * ms                     # AMPA Decay Time constant
taugr =  0.25 * ms                  # GABA Rise Time constant
tauar =  0.4 * ms                   # AMPA Rise Time constant
tau_stp = 1000 * ms                 # Short-term Plasticity Time constant


K_GABA = 10                         # GABA currnets' multiplier
K_Mod = 3 /mV*ms                    # Modulation currnets' multiplier
K_Tac = 30 * mV/ms                  # Tactile Input currnets' multiplier
K_Prop = 30 * mV/ms                 # Proprioceptive Input currnets' multiplier

K_AMPA = 50                         # AMPA currnets' multiplier
K_AG = 1                            # GABA and AMPA currnets' multiplier


###########################################################
#############     Synapse parameters        ###############
###########################################################

taupre = 5*ms                       # STDP Pre Trace Time Constant
taupost = 60*ms                     # STDP Post trace Time Constant
wmax = 150 * mV / ms                # Maximum Weight
wmin = 0 * mV / ms                  # Minimum Weight
Apre = 0.01                         # STDP Pre Trace increase
Apost = -Apre*taupre/taupost*1.5    # STDP Pre Trace increase
L_Rate = 0#10                       # Learning Rate