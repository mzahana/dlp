# DLP
Implementation of Distributed Linear Program in Multi-agent Pursuit-Evasion Game
# Installation
If you are going to use Pyhon version, you need to install GLPK, and CVXOPT toolboxes. Otherwise, only GLPK is required.
GLPK is already included in this repo. To install it,
* ```cd``` to ```glpk``` folder and run the folloing commands in sequence,
```
./configure
make # to compile GLPK Libs
make check # to make sure everything compiled correctly
make install # to install Libs in the default system paths
```
**NOTE**: you might need to use ```sudo make install``` instead of ```make install```, if you get error ```permission denied```.
Then go to the 'Using C++' section below to test DLP.
**NOTE** compiling DLP test files fails for some reason, try to clean the glpk installation, and re-do it without compliling the shared libs,
```
# run inside glpk folder
sudo make uninstall
make clean
make distclean
# re-configure without share libs, and compile/install again
./configure --disable-shared
make # compile
make check # running test files to check if GLPK runs OK!
make install
```
Then, try to  compile DLP test files again, following the below section.

# Using C++
* make sure to install GLPK in the default paths
* run,
```
make testDLP
./bin/testDLP
```
* to clean,
```
make clean
```

# Using Python
* make sure you install ```cvxopt``` toolbox
* check the ```test.py``` for a template
* to run,
```
python scripts/test.py
```
