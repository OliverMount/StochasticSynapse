from neuron import h
import numpy as np
import matplotlib.pyplot as plt
import random

h.load_file("stdrun.hoc")

# Simulation params
n_cells = 5
tstop = 300
release_prob = 0.1

# Create shared NetStim
stim = h.NetStim()
stim.number = 100
stim.start = 5
stim.interval = 10
stim.noise = 0.03

# Storage
somas = []
syns = []
netcons = []
voltages = []

# Record time after at least one section exists
times = None

for i in range(n_cells):
    # Create a soma section
    soma = h.Section(name=f"soma_{i}")
    soma.L = soma.diam = 10
    soma.insert('hh')
    somas.append(soma)
    
    # Create synapse
    syn = h.StochExp2Syn(soma(0.5)) 
    syn.tau1 = 0.1
    syn.tau2 = 2
    #syn.e = 0
    syn.release_prob = release_prob

    # Assign different random seeds
    spatial_seed = random.randint(100, 1000)
    temporal_seed = random.randint(10, 999)
    syn.setSeeds(spatial_seed, temporal_seed)
    syns.append(syn)

    # Connect to same NetStim
    nc = h.NetCon(stim, syn)
    nc.weight[0] = 0.004
    netcons.append(nc)

    # Record voltage
    v_vec = h.Vector().record(soma(0.5)._ref_v)
    voltages.append(v_vec)

    # Record time once
    if times is None:
        times = h.Vector().record(h._ref_t)

# Run
h.finitialize(-65)
h.continuerun(tstop)

  
# Plot with vertical offsets
plt.figure(figsize=(12, 6))
offset = 30  # vertical spacing in mV

for i, v in enumerate(voltages):
    v_shifted = np.array(v) + i * offset  # apply vertical offset
    plt.plot(times, v_shifted, label=f"Cell {i+1}")

plt.xlabel("Time (ms)",fontsize=14)
plt.ylabel("Membrane Voltage + Offset (mV)",fontsize=14)
plt.title(f" Release Probability : {release_prob} ",fontsize=16 )
plt.yticks([])  # optional: hide y-axis ticks
plt.legend()
plt.grid(True)
plt.tick_params(axis='both', labelsize=12) 
plt.tight_layout()
plt.show() 


#Non-Overlapping Traces of 5 Cells to Shared Stochastic Input
