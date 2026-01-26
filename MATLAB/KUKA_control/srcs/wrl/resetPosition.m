kuka_wrl = vrworld('KUKAKR6.wrl');
open(kuka_wrl);
view(kuka_wrl);

homeConfig = [0, 0, 0, 0, 0, 0];
kuka_wrl.shoulder.rotation  = [0, 1, 0, homeConfig(1)];
kuka_wrl.elbow.rotation     = [0, 0, 1, homeConfig(2)];
kuka_wrl.pitch.rotation     = [0, 0, 1, homeConfig(3)];
kuka_wrl.roll.rotation      = [1, 0, 0, homeConfig(4)];
kuka_wrl.yaw.rotation       = [0, 0, 1, homeConfig(5)];
kuka_wrl.drill.rotation     = [1, 0, 0, homeConfig(6)];