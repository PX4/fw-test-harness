# Fixedwing Test Harness

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

## Run
```
cd fw_test_harness
python2 simulator.py
```

# some related .profile entries (need to improve packaging)
```
export PYTHONPATH=$SRCDIR/fw-test-harness/external/jsbsim/build/tests:$PYTHONPATH
export PYTHONPATH=$SRCDIR/fw-test-harness/build/fw_test_harness/build/lib.linux-x86_64-2.7:$PYTHONPATH
export LD_LIBRARY_PATH=$SRCDIR/fw-test-harness/build:$LD_LIBRARY_PATH
```

