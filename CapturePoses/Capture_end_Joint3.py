#Magician
from numbers import Number

Xpos = None
Ypos = None
Zpos = None
Rpos = None
X = None


Xpos = 0
Ypos = 0
Zpos = 0
Rpos = 0
X = 80

current_pose = dType.GetPose(api)
dType.SetPTPCmdEx(api, 4, (0),  0,  0, current_pose[7], 1)
for count in range(100):
  X = (X if isinstance(X, Number) else 0) + 0.1
  current_pose = dType.GetPose(api)
  dType.SetPTPCmdEx(api, 4, 0,  45,  X, current_pose[7], 1)
  Xpos = str(Xpos) + str(',')
  Xpos = str(Xpos) + str(dType.GetPoseEx(api, 1))
  Ypos = str(Ypos) + str(',')
  Ypos = str(Ypos) + str(dType.GetPoseEx(api, 2))
  Zpos = str(Zpos) + str(',')
  Zpos = str(Zpos) + str(dType.GetPoseEx(api, 3))
  Rpos = str(Rpos) + str(',')
  Rpos = str(Rpos) + str(dType.GetPoseEx(api, 4))
print(Xpos)
print(Ypos)
print(Zpos)
print(Rpos)
