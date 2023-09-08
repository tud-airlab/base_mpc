# base_mpc
navigation for AirLab

## Overview

## Installation
We provide detailed installation instructions [here](docs/installation.md).

## Generate MPC solver
We currently use ForcesPro, a fast and realiable numerical solver for mathematical optimization problems. For academic use the license is for free.
More information on the installation of ForcesPro can be found [here](docs/installation.md).

After getting the license and setting up ForcesPro follow the steps below to generate the MPC solver.

          cd base_mpc/python_solver_generation
          poetry install
          poetry shell
          python base_mpc_solver.py
          
Tipp: see output for information on which parameter corresponds to which variable

This generates a folder named 'AlbertFORCESNLPsolver' in python_solver_generation.

[ToDo: Extension for Acados]



