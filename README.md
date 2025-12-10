# Environment Setup
Fetch submodules:
```bash
git submodule update --init --recursive
```

Create conda environment:
```bash
conda env create -f environment.yml -n gz-env
```

Activate conda environment:
```bash
conda activate gz-env
```

Build ArduPilot:
```bash
cd ardupilot
./waf configure --board=sitl
./waf plane
```

Build ardupilot\_gazebo plugin:
```bash
cd ardupilot_gazebo
mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Release ..
ninja -j $(nproc)
```

Patch ArduPilot SITL to add V-tail model:
```bash
cd ardupilot
git apply ../vtail.patch
```

# Training
To train, run in one window to start SITL:
```bash
cd ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-mini-talon-vtail --model JSON --map --console
```

In another window:
```bash
python src/PPO.py
```

# Testing
To test, run in one window to start SITL:
```bash
cd ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-mini-talon-vtail --model JSON --map --console
```

In another window:
```bash
python src/test.py
```
