# Izhikivich Neuron model with a Timmed array, GABA, and AMPA current inputs

# These equations define the dynamics of the membrane potential v and the recovery variable u, 
# as well as the input current that the neuron receives.



from brian2 import *

# Define the equations for the Prop Neurons
EQ_Prop_1 = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + I_P + K_Prop * I_P1           : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I_P = K_Prop * I_Prop_1(t-segment_offset,i)                                      : volt/second
segment_offset                                                                   : second
I_P1                                                                             : 1
''' 

# Define the equations for the Prop Neurons
EQ_Prop_2 = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + I_P + K_Prop * I_P1           : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I_P = K_Prop * I_Prop_2(t-segment_offset,i)                                      : volt/second
segment_offset                                                                   : second
I_P1                                                                             : 1
'''

# Define the equations for the Target Prop Neurons
EQ_Prop_T1 = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + I + K_Prop * I_P1             : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I = K_GABA * Ig + K_AMPA * Ia                                                    : volt/second
dIg/dt = (-Ig+Ig1)/taugd                                                         : volt/second
dIa/dt = (-Ia+Ia1)/tauad                                                         : volt/second
dIg1/dt = -Ig1/taugr                                                             : volt/second
dIa1/dt = -Ia1/tauar                                                             : volt/second
I_P1                                                                             : 1
''' 

# Define the equations for the Target Prop Neurons
EQ_Prop_T2 = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + I + K_Prop * I_P1             : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I = K_GABA * Ig + K_AMPA * Ia                                                    : volt/second
dIg/dt = (-Ig+Ig1)/taugd                                                         : volt/second
dIa/dt = (-Ia+Ia1)/tauad                                                         : volt/second
dIg1/dt = -Ig1/taugr                                                             : volt/second
dIa1/dt = -Ia1/tauar                                                             : volt/second
I_P1                                                                             : 1
'''

# Define the equations for the Tactile Neurons
EQ_Tactile = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + I_T + I                       : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I_T = K_Tac * I_Tactile(t-segment_offset,i)                                      : volt/second
segment_offset                                                                   : second
I = K_GABA * Ig + K_AMPA * Ia                                                    : volt/second
dIg/dt = (-Ig+Ig1)/taugd                                                         : volt/second
dIa/dt = (-Ia+Ia1)/tauad                                                         : volt/second
dIg1/dt = -Ig1/taugr                                                             : volt/second
dIa1/dt = -Ia1/tauar                                                             : volt/second
'''

  
# Define the equations for the Spatial Gain Field Neurons
EQ_GF = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + K_AG * I + I2                 : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I = K_Mod * Ia_P1 * Ia_P2 * I_Tac(t-segment_offset,i)                            : volt/second
dIa_P2/dt = (-Ia_P2+Ia1_P2)/tauad                                                : volt/second
dIa1_P2/dt = -Ia1_P2/tauar                                                       : volt/second
dIa_P1/dt = (-Ia_P1+Ia1_P1)/tauad                                                : volt/second
dIa1_P1/dt = -Ia1_P1/tauar                                                       : volt/second
X                                                                                : 1
Y                                                                                : 1
segment_offset                                                                   : second
I2 = K_GABA * Ig + K_AMPA * Ia                                                   : volt/second
dIg/dt = (-Ig+Ig1)/taugd                                                         : volt/second
dIa/dt = (-Ia+Ia1)/tauad                                                         : volt/second
dIg1/dt = -Ig1/taugr                                                             : volt/second
dIa1/dt = -Ia1/tauar                                                             : volt/second
'''


# Define the equations for the Motor Gain Field Neurons
EQ_MGF = '''
dv/dt = (0.04/ms/mV)*v**2+(5/ms)*v+140*mV/ms - u + K_AG * I + I2                 : volt (unless refractory)
du/dt = a*(b*v-u)                                                                : volt/second
I = K_Mod * Ia_P1 * Ia_P2                                                        : volt/second
dIa_P2/dt = (-Ia_P2+Ia1_P2)/tauad                                                : volt/second
dIa1_P2/dt = -Ia1_P2/tauar                                                       : volt/second
dIa_P1/dt = (-Ia_P1+Ia1_P1)/tauad                                                : volt/second
dIa1_P1/dt = -Ia1_P1/tauar                                                       : volt/second
X                                                                                : 1
Y                                                                                : 1
segment_offset                                                                   : second
I2 = K_GABA * Ig + K_AMPA * Ia                                                   : volt/second
dIg/dt = (-Ig+Ig1)/taugd                                                         : volt/second
dIa/dt = (-Ia+Ia1)/tauad                                                         : volt/second
dIg1/dt = -Ig1/taugr                                                             : volt/second
dIa1/dt = -Ia1/tauar                                                             : volt/second
'''


### Simple Excitatory STDP Synapse

###########################################################
############        Synapse Equations        ##############
###########################################################

Syn_eq1 =    '''
             w : volt/second
             dapre/dt = -apre/taupre            : 1 (event-driven)
             dapost/dt = -apost/taupost         : 1 (event-driven)
             dw_stp/dt = -(w_stp/tau_stp)       : 1 (event-driven)
             '''
EX_on_pre_eq =  '''
                Ia1 += w / (1+w_stp)
                apre += Apre
                w = clip(w+(L_Rate*apost)*volt/second, wmin, wmax)
                '''
EX_on_post_eq = '''
                apost += Apost
                
                w = clip(w+(L_Rate*apre)*volt/second, wmin, wmax)
                '''



            #  dw/dt = -w/(1000*second) : volt/second
            # w_stp += 1