from neuron import h
import numpy as np
import matplotlib.pyplot as plt
import random
import os

h.load_file("stdrun.hoc")

# Simulation params
n_cells = 5
tstop = 150

# Ensure save folder exists
os.makedirs("../figs", exist_ok=True)


def run_simulation(release_prob):  
    stim = h.NetStim()
    stim.number = 100
    stim.start = 2
    stim.interval = 10
    stim.noise = 0.3

    somas, syns, netcons, voltages = [], [], [], []
    times = None

    for i in range(n_cells):
        # Soma
        soma = h.Section(name=f"soma_{i}")
        soma.L = soma.diam = 10
        soma.insert("hh")
        somas.append(soma)

        # Synapse (our custom GPU stochastic synapse)
        syn = h.StochExp2SynGPU(soma(0.5))
        syn.tau1 = 0.1
        syn.tau2 = 2
        syn.e=0
        syn.release_prob = release_prob

        # Random seeds for stochasticity 
        seed1 = random.randint(1, 10000)
        seed2 = random.randint(1, 10000)
        seed3 = random.randint(1, 10000)
        syn.setSeeds(seed1, seed2, seed3)
        syns.append(syn)

        # NetCon from shared NetStim
        nc = h.NetCon(stim, syn)
        nc.weight[0] = 0.05
        netcons.append(nc)

        # Record voltage trace
        v_vec = h.Vector().record(soma(0.5)._ref_v)
        voltages.append(v_vec)

        # Record time only once
        if times is None:
            times = h.Vector().record(h._ref_t)

    # Run simulation
    h.finitialize(-65)
    h.continuerun(tstop)

    return times, voltages


# Run for different release probabilities
release_probs = [1.0, 0.5, 0.3, 0.1]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten()

offset = 30  # vertical spacing in mV

for ax, rp in zip(axes, release_probs):
    times, voltages = run_simulation(rp)

    for i, v in enumerate(voltages):
        v_shifted = np.array(v) + i * offset
        ax.plot(times, v_shifted, label=f"Cell {i+1}")

    ax.set_xlabel("Time (ms)", fontsize=12)
    ax.set_ylabel("Membrane Voltage + Offset (mV)", fontsize=12)
    ax.set_title(f"Release Probability: {rp}", fontsize=14)
    ax.set_yticks([])
    ax.tick_params(axis="both", labelsize=10)
    ax.grid(True)
    ax.legend(fontsize=8)

plt.tight_layout()

# Save to file
save_path = "../figs/CoreNEURONtest.png"
plt.savefig(save_path, dpi=300)
plt.close()

print(f"Figure saved as {save_path}")

