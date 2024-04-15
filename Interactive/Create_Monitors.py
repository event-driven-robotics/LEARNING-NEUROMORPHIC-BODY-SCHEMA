from brian2 import *


###############################################################
#############        Creat Spike Monitors        ##############
###############################################################

def Create_SP_Monitors(Prop_Neurons_1, Prop_Neurons_2,Prop_Neurons_T1, Prop_Neurons_T2, GF_Neurons, Tactile_Neurons, Directional_M1_Neurons, Directional_M2_Neurons):

    ###########################
    # Create Prop_Neurons Monitors 
    SP_Prop_1 = SpikeMonitor(Prop_Neurons_1)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_Prop_2 = SpikeMonitor(Prop_Neurons_2)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_Prop_T1 = SpikeMonitor(Prop_Neurons_T1)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_Prop_T2 = SpikeMonitor(Prop_Neurons_T2)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_GF = SpikeMonitor(GF_Neurons)
    ###########################

    ###########################
    # Create Tactile_Neurons Monitors 
    SP_Tactile = SpikeMonitor(Tactile_Neurons)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_M1 = SpikeMonitor(Directional_M1_Neurons)
    ###########################

    ###########################
    # Create Prop_Neurons Monitors 
    SP_M2 = SpikeMonitor(Directional_M2_Neurons)
    ###########################   

    return SP_Prop_1, SP_Prop_2, SP_Prop_T1, SP_Prop_T2, SP_GF, SP_Tactile, SP_M1, SP_M2


###############################################################
##########      Creat Neuron's State Monitors       ###########
###############################################################

def Create_Neuron_ST_Monitors(Prop_Neurons, Tactile_Neurons, Bimodal_Neurons):

    ###########################
    # Create Prop_Neurons Monitors 
    ST_Prop = StateMonitor(Prop_Neurons, 'v', record=True)
    ###########################

    ###########################
    # Create Tactile_Neurons Monitors 
    ST_Tactile = StateMonitor(Tactile_Neurons, ['v', 'I_T'], record=True)
    ###########################

    ###########################
    # Create Ret_Bimodal_Neurons Monitors 
    ST_Bimomdal = StateMonitor(Bimodal_Neurons, 'v', record=True)
    ###########################

  
    return ST_Prop, ST_Tactile, ST_Bimomdal


###############################################################
##########      Creat Synapse's State Monitors       ###########
###############################################################

def Create_Synapse_ST_Monitors(GF_Tactile):

    ###########################
    # Create Prop_GF Synapses Monitors 
    ST_GF_TAC = StateMonitor(GF_Tactile, ['w', 'w_stp'], record=True)
    ###########################

    ###########################
    # Create Tac_GF Synapses Monitors  
    # ST_Tac_Bi = StateMonitor(Tac_BI, variables=True, record=True)
    ###########################


    return ST_GF_TAC



###############################################################
##########        Create Test Spike Monitors        ###########
###############################################################

def Create_Test_SP_Monitors(Vision_Neurons, Ret_Tac_Neurons):

    ###########################
    # Create Imagined Vision Monitors  
    SP_Vision = SpikeMonitor(Vision_Neurons)
    ###########################
    
    ###########################
    # Create Real Retina Monitors  
    SP_Ret_Tac_Neurons = SpikeMonitor(Ret_Tac_Neurons)
    ###########################

    return SP_Vision, SP_Ret_Tac_Neurons