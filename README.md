# Monte Carlo Simulation of Light Propagation in Turbid Tissue

![alt text](https://github.com/The4ofUs/Monte-Carlo-Simulation-of-Light-Propagation/blob/master/MonteCarlo-Simulation.png?raw=true "Simulation Results")

## How to start the Simulation

### Requirements
1. A NVIDIA with CUDA Support GPU. [Check here](https://en.wikipedia.org/wiki/CUDA)
2. CUDA toolkit and driver. [Download](https://developer.nvidia.com/cuda-downloads)
3. CMake Version >= 3.10.

### Build

#### Simulation
1. Open `/Simulation/CMakeLists.txt`, instructions on changes that is architectural dependent is commented there.
2. After changing the required lines in the `CMakeLists.txt`, Check `/Simulation/code/src/MC_Simulation.cu` for simulation parameters editing. You should pick a suitable `THREADS_PER_BLOCK` number. That is totally dependent on your GPU's architecture.
3. Build on!
4. Run `/<build-directory>/MC_Simulation` to execute the simulation.
5. Results should be found at `/<build-directory>/Results.csv`, but you can edit the streaming out to any format you want from `/Simulation/code/src/MC_Helpers.cu`.

#### Network  

##### Server side
1. Open `/Network/Server/serverSide/src/Thread.cpp` to change simulation parameters.
2. Open `/Network/Server/serverSide/src/TcpServer.cpp` to change the numbers of `serverBucketOfPhotons` and `photonsPerBatch`.
3. Build on!
4. Run `/<build-directory>/ServerSide` to execute the server side algorithm.
5. Start listening from the UI.
6. Results should be found at `/<build-directory>/serverReceivedResults.csv`, but you can edit the streaming out to any format you want from `/Network/Server/serverSide/src/TcpServer.cpp`.

##### Client side
1. Open `/Network/Client/CMakeLists.txt`, instructions on changes that is architectural dependent is commented there.
2. After changing the required lines in the `CMakeLists.txt`, Check `/Network/code/src/MC_Simulation.cu` to pick a suitable `THREADS_PER_BLOCK` number. That is totally dependent on your GPU's architecture.
3. Open `/Network/Client/clientSide/src/socket.cpp` to change the IP of the server.
4. Build on!
5. Wait for the server to start listening, then run `/<build-directory>/MC_Simulation` to execute the client side algorithm.
6. Results should be found at `/<build-directory>/clientSentPhotons.csv`, but you can edit the streaming out to any format you want from `/Network/Client/code/src/MC_Helpers.cu`.

### Checksum
* A sample of the results is accompanied with the repository, once you started producing similar results, consider the Simulation wholesome.

### Gadgets
* You can use our plotter to check for your results' distribution right [here](https://github.com/The4ofUs/MonteCarlo-Plotter).

### More info
Check our [Paper](https://drive.google.com/file/d/1qz25djuWe6Q_Mj5KZSe2H7L1YjMzodiI/view?usp=sharing) and our [Thesis](https://drive.google.com/file/d/1SdsYElqOyyaozrBfbyexvN9XPPBtcGir/view?usp=sharing) for further thorough documentation.
