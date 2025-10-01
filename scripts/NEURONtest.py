from neuron import h
import numpy as np
import matplotlib.pyplot as plt
import random
import os

h.load_file("stdrun.hoc")

# Simulation params
n_cells = 5
tstop = 150

# Make sure save folder exists
os.makedirs("../figs", exist_ok=True)

def run_simulation(release_prob): 
    # Shared NetStim
    stim = h.NetStim()
    stim.number = 70
    stim.start = 2
    stim.interval = 10
    stim.noise = 0.03

    somas, syns, netcons, voltages = [], [], [], []
    times = None

    for i in range(n_cells):
        # Soma
        soma = h.Section(name=f"soma_{i}")
        soma.L = soma.diam = 10
        soma.insert('hh')
        somas.append(soma)

        # Synapse
        syn = h.StochExp2Syn(soma(0.5))
        syn.tau1 = 0.1
        syn.tau2 = 2
        syn.release_prob = release_prob

        # Random seeds for stochasticity
        spatial_seed = random.randint(100, 1000)
        temporal_seed = random.randint(10, 999)
        syn.setSeeds(spatial_seed, temporal_seed)
        syns.append(syn)

        # NetCon
        nc = h.NetCon(stim, syn)
        nc.weight[0] = 0.004
        netcons.append(nc)

        # Record voltages
        v_vec = h.Vector().record(soma(0.5)._ref_v)
        voltages.append(v_vec)

        # Record time once
        if times is None:
            times = h.Vector().record(h._ref_t)

    # Run simulation
    h.finitialize(-65)
    h.continuerun(tstop)

    return times, voltages


# Run for different release probabilities
release_probs = [1, 0.5, 0.3, 0.1]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten()

offset = 30  # mV spacing for stacked plots

for ax, rp in zip(axes, release_probs):
    times, voltages = run_simulation(rp)

    for i, v in enumerate(voltages):
        v_shifted = np.array(v) + i * offset
        ax.plot(times, v_shifted, label=f"Cell {i+1}")

    ax.set_xlabel("Time (ms)", fontsize=12)
    ax.set_ylabel("Membrane Voltage + Offset (mV)", fontsize=12)
    ax.set_title(f"Release Probability: {rp}", fontsize=14)
    ax.set_yticks([])
    ax.tick_params(axis='both', labelsize=10)
    ax.grid(True)
    ax.legend(fontsize=8)

plt.tight_layout()

# Save to file
save_path = "../figs/release_prob_traces.png"
plt.savefig(save_path, dpi=300)
plt.close()

print(f"Figure saved as {save_path}")

