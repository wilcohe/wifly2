# wifly2

This repo is the back-end for the Wifly navigation algorithm. It provides an occupancy map and intensity data for the nearest point to the drone, allowing us to simulate our algorithm based on a few assumptions. They are as below. 

1. The wifi is a circular signal that decays as 1/r
2. The signal experiences a seperate level of decay through a wall cell, and we will see higher RSS attenuation as a result
3. The signal is constant in time
