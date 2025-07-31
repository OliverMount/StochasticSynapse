# Stochastic Synapse
NEURON Simulator Stochastic Synapse

This repository extends the conventional  NEURONSs' `Exp2Syn` synapse model to incorporate stochastic vesicle release, providing a more biologically realistic representation of synaptic transmission. Our implementation supports both `NEURON` and `CoreNEURON`, ensuring compatibility with GPU-accelerated and large-scale network simulations.

We introduce stochasticity using a release probability mechanism, enabling diverse response patterns across targets—even with shared presynaptic input. The model can be used for both excitatory and inhibitory synapses, and integrates seamlessly with Python-based simulations.

This work is designed to aid researchers exploring probabilistic synaptic dynamics in both small and large network contexts.
