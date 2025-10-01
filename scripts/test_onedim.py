from neuron import h
import numpy as np
import matplotlib.pyplot as plt
import random   

h.load_file("stdrun.hoc")  
soma=h.Section(name='soma')
soma.L=soma.diam=10
soma.insert('hh')


syn=h.StochExp2Syn(soma(0.5))
syn.tau1=0.1
syn.tau2=2
syn.e=0
syn.release_prob=0.5


spatial_seed1=300
temporal_seed1=1000 

spatial_seed2=34
temporal_seed2=98

syn.setSeeds(random.randint(spatial_seed1,temporal_seed1),random.randint(spatial_seed2,temporal_seed2))

stim=h.NetStim()
stim.number=100
stim.start=5
stim.interval=10
stim.noise=0.03

nc=h.NetCon(stim,syn)
nc.weight[0]=0.004


# Record synaptic current and membrane voltage
t_vec = h.Vector().record(h._ref_t)
v_vec = h.Vector().record(soma(0.5)._ref_v)
i_vec = h.Vector().record(syn._ref_i)

# Run simulation
h.finitialize(-65)
h.continuerun(150)

# Plot
plt.figure(figsize=(10, 4))
plt.plot(t_vec, v_vec, label="Membrane Voltage (mV)")
plt.plot(t_vec, i_vec, label="Synaptic Current (nA)")
plt.legend()
plt.xlabel("Time (ms)")
plt.ylabel("Response")
plt.title("StochExp2Syn Test: release_prob = {:.2f}".format(syn.release_prob))
plt.grid(True)
plt.show()
