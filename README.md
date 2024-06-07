
# Franka Easy Inverse Kinematics

### Install
```
pip3 install roboticstoolbox-python
git clone git@github.com:petrvancjr/franka_easy_ik.git
cd franka_easy_ik
pip3 install .
```

### Usage

```python
from franka_easy_ik import FrankaEasyIK
ik = FrankaEasyIK()

position = [0.5,0.,0.3] # x, y, z
orientation = [1., 0., 0., 0.] # x, y, z, w
q = ik(position, orientation)
print(q)
```
