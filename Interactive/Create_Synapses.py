from Equations import *
# from Weight_dependant_E_STDP import *
from brian2 import *
import Functions as FN



###############################################################
#############     Create and Load Synapses        #############
###############################################################

def Load_Synapses(Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, GF_Neurons, Tactile_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons):


    ###########################
    # Create Propriception 1 to Gain-Field Synapses 
    Prop1_GF = Synapses(Prop_Neurons_1, GF_Neurons, ''' w : volt/second''', on_pre='''Ia1_P1 +=w''', method='euler', delay=0 * ms)
    Prop1_GF.connect(condition='i==X')#condition='j==i')
    Prop1_GF.w = 140 * mV / ms
    ###########################

    ###########################
    # Create Propriception 2 to Gain-Field Synapses 
    Prop2_GF = Synapses(Prop_Neurons_2, GF_Neurons, ''' w : volt/second''', on_pre='''Ia1_P2 +=w''', method='euler', delay=0 * ms)
    Prop2_GF.connect(condition='i==Y')#condition='j==i')
    Prop2_GF.w = 140 * mV / ms
    ###########################

    ###########################
    # Create Gain-Field to target proprioceptive 1 Synapses 
    GF_Prop1 = Synapses(GF_Neurons, Prop_Neurons_T1, ''' w : volt/second''', on_pre='''Ia1 +=w''', method='euler', delay=0 * ms)
    GF_Prop1.connect(condition='j==X_pre')#condition='j==i')
    GF_Prop1.w = 0.7 * mV / ms
    ###########################

    ###########################
    # Create Gain-Field to target proprioceptive 2 Synapses 
    GF_Prop2 = Synapses(GF_Neurons, Prop_Neurons_T2, ''' w : volt/second''', on_pre='''Ia1 +=w''', method='euler', delay=0 * ms)
    GF_Prop2.connect(condition='j==Y_pre')#condition='j==i')
    GF_Prop2.w = 0.7 * mV / ms
    ###########################

    ###########################
    # Create Gain-Field to Tactile Synapses 
    GF_Tactile = Synapses(GF_Neurons, Tactile_Neurons, Syn_eq1 , on_pre=EX_on_pre_eq, on_post=EX_on_post_eq, method='euler', delay=0 * ms)
    GF_Tactile.connect()
    GF_Tactile.w = 0 * mV / ms
    SynapseDir = './Trained_Weights/'
    Syn_Name = 'GF_Tactile'
    Syn_Suff = '_TR1_24_11_epoch400'
    W = np.load(SynapseDir+Syn_Name+Syn_Suff+'.npy')
    W = np.interp(W, [0, max(W)], [0, 30])
    GF_Tactile.w =  W * mV / ms
    GF_Tactile.w_stp = 0

    ###########################
    # Create Backeard connection form Tactile Neurons to GF Neurons
    for_pre_ind = GF_Tactile.i
    for_post_ind = GF_Tactile.j
    for_weight = GF_Tactile.w
    for_weight = np.interp(for_weight, [0, max(for_weight)], [0, 30])

    Tactile_GF = Synapses(Tactile_Neurons, GF_Neurons, ''' w : volt/second''' , on_pre='''Ia1 +=w''',  method='euler', delay=0 * ms)
    Tactile_GF.connect()

    for k in range(len(for_pre_ind)):
        Tactile_GF.w[for_post_ind[k], for_pre_ind[k]] = for_weight[k]*mV/ms
    # Tactile_GF.w_stp = 0
        
    ###########################
    # Create Propriception to Motor_Gain-Field_1 Synapses 
    Prop1_MGF = Synapses(Prop_Neurons_1, MGF_Neurons_1, ''' w : volt/second''', on_pre='''Ia1_P1 +=w''', method='euler', delay=0 * ms)
    Prop1_MGF.connect(condition='i==X')
    Prop1_MGF.w = 140 * mV / ms
    ###########################
    TProp1_MGF = Synapses(Prop_Neurons_T1, MGF_Neurons_1, ''' w : volt/second''', on_pre='''Ia1_P2 +=w''', method='euler', delay=0 * ms)
    TProp1_MGF.connect(condition='i==Y')
    TProp1_MGF.w = 100 * mV / ms
    ###########################


    ###########################
    # Create Propriception to Motor_Gain-Field_2 Synapses 
    Prop2_MGF = Synapses(Prop_Neurons_2, MGF_Neurons_2, ''' w : volt/second''', on_pre='''Ia1_P1 +=w''', method='euler', delay=0 * ms)
    Prop2_MGF.connect(condition='i==X')
    Prop2_MGF.w = 140 * mV / ms
    ###########################
    TProp2_MGF = Synapses(Prop_Neurons_T2, MGF_Neurons_2, ''' w : volt/second''', on_pre='''Ia1_P2 +=w''', method='euler', delay=0 * ms)
    TProp2_MGF.connect(condition='i==Y')
    TProp2_MGF.w = 100 * mV / ms
    ###########################

    ###########################
    # Create Motor_Gain-Field_1 to Directional Motoneurons Synapses 
    MGF1_DM1 = Synapses(MGF_Neurons_1, Directional_M1_Neurons, ''' w : volt/second''', on_pre='''Ia1 +=w''', method='euler', delay=0 * ms)
    MGF1_DM1.connect(condition='(X_pre-Y_pre)<0 and j==1')
    MGF1_DM1.connect(condition='(X_pre-Y_pre)>0 and j==0')
    MGF1_DM1.w = 140 * mV / ms

    ###########################
    ###########################
    # Create Motor_Gain-Field_2 to Directional Motoneurons Synapses 
    MGF2_DM2 = Synapses(MGF_Neurons_2, Directional_M2_Neurons, ''' w : volt/second''', on_pre='''Ia1 +=w''', method='euler', delay=0 * ms)
    MGF2_DM2.connect(condition='(X_pre-Y_pre)<0 and j==1')
    MGF2_DM2.connect(condition='(X_pre-Y_pre)>0 and j==0')
    MGF2_DM2.w = 140 * mV / ms

    ###########################
    # Create GF Inhibitory Synapses 
    GF_GF_Inh = Synapses(GF_Neurons, GF_Neurons, ''' w : volt/second''', on_pre='''Ig1 -=w''', method='euler', delay=2 * ms)
    GF_GF_Inh.connect(condition='i!=j')
    GF_GF_Inh.w = 15 * mV / ms
    # GF_GF_Inh.connect(condition='sqrt((X_pre-X_post)**2+(Y_pre-Y_post)**2)>3')
    # GF_GF_Inh.w = '(200 * exp(-(sqrt((X_pre-X_post)**2+(Y_pre-Y_post)**2)-10)**2/3)) * mV / ms'
    GF_GF_Inh.w = 0 * mV / ms
    ###########################

    ###########################
    # Create GF Inhibitory Synapses 
    GF_GF_Ex = Synapses(GF_Neurons, GF_Neurons, ''' w : volt/second''', on_pre='''Ia1 +=w''', method='euler', delay=2 * ms)
    # GF_GF_Ex.connect(condition='i==j')
    # GF_GF_Ex.w = 20 * mV / ms
    GF_GF_Ex.connect()
    GF_GF_Ex.w = '10 * exp(-(sqrt((X_pre-X_post)**2+(Y_pre-Y_post)**2))/1) * mV / ms'
    GF_GF_Ex.w = 0 * mV / ms
    ###########################

    return Prop1_GF, Prop2_GF,GF_Prop1, GF_Prop2, GF_Tactile, Tactile_GF, Prop1_MGF, Prop2_MGF, TProp1_MGF, TProp2_MGF, MGF1_DM1, MGF2_DM2, GF_GF_Inh, GF_GF_Ex

