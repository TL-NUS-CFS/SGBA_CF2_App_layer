## SGBA App layer

Hi and welcome at the Crazyflie firmware app layer implementation of the Swarm Gradient Bug Algorithm (SGBA), accomendating the publication of:

> K.N. McGuire, C. De Wagter, K. Tuyls, H.J. Kappen, G.C.H.E. de Croon,
> 'Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment'
> Science Robotics, 23 October 2019 
> DOI: http://robotics.sciencemag.org/lookup/doi/10.1126/scirobotics.aaw9710

This repo will undergo several updates in the coming period, so please keep checking the status. Also if you have any questions, please use the issue list of this repository.

**Please beware that this code is not the actual source code with which the Crazyflies in the publication has flown with. This is just an updated firmware to showcase the new applayer and peer2peer communication. If you want the old original firmware code that actually flew in the real-world experiments as in the paper, go to https://github.com/tudelft/SGBA_code_SR_2019**

#### Build instructions

Make sure your crazyflie has the newest NRF firmware with bluetooth disabled! (https://github.com/bitcraze/crazyflie2-nrf-firmware). Also give your crazyflies different address with an unique ID: E7E7E701, E7E7E702, etc. and make sure they are listining all on the same channel. Maximum is until 09 for now.

#### Clone this repo

```
git clone --recursive https://github.com/thisRepoAddress
cd SGBA_CF2_App_layer/
git checkout dev/compilable
git submodule sync
git submodule init
git submodule update  --recursive
```

#### Build and flash
Make sure to do this in the directory of the applayer (this repo) and not the directory of the crazyflie firmware)

```
make clean
make
make cload
```

#### Scripts
Use 2 crazyradio PAs: 
 - 1 to make the crazyflies take off (python3 take_off.py) and land (python3 land_all.py). Use this as helper scripts, but make sure to close take_off.py once all crazyflies has taken off.
 - 1 to act as the home beacon (python3 RSSI_beacon.py)

