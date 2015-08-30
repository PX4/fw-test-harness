# Fixed-wing Test Harness
A simulation wrapper around the PX4 fixed-wing control stack. Flight dynamics is simulated with jsbsim. A html report with interactive graphs is generated after the simulation to which new plots can be added quickly.

The simulation is currently using the standard px4 attitude controller together with mTECS for altitude/speed control.

## Setup
### Archlinux
```
./setup_archlinux.sh
```

# Build
## Firmware
```
mkdir build
cd build
cmake ..
make
```

# Run
```
cd fw_test_harness
python2 simulator.py
```

# Add a plot
See the output_results function in simulator.py

# some related .profile entries (need to improve packaging)
```
export PYTHONPATH=$SRCDIR/fw-test-harness/external/jsbsim/build/tests:$PYTHONPATH
export PYTHONPATH=$SRCDIR/fw-test-harness/build/fw_test_harness/build/lib.linux-x86_64-2.7:$PYTHONPATH
export LD_LIBRARY_PATH=$SRCDIR/fw-test-harness/build:$LD_LIBRARY_PATH
```

