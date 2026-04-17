# Battery SOC Estimation using Factor Graph Optimization (GTSAM)

## Overview
This project implements battery State of Charge (SOC) estimation using Factor Graph Optimization (FGO) with the GTSAM library.

## Structure

- main.cpp  
  Entry point for SOC estimation

- SOCProcessFactor  
  Custom factor for SOC state transition

- data/  
  Input data (current, voltage, etc.)

- run_logs/  
  Output logs and results

## Build

```bash
mkdir build
cd build
cmake ..
make
