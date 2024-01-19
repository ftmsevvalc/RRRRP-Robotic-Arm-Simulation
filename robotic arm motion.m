robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
%robot bileşenlerini oluşturur:
rotatingBase = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
gripper = rigidBody("gripper");
collBase = collisionCylinder(0.05,0.04); % cylinder: radius,length
collBase.Pose = trvec2tform([0 0 0.04/2]); % 3D dönüşüm matrisleri
coll1 = collisionCylinder(0.025,0.1); % cylinder: radius,length
coll1.Pose = trvec2tform([0 0 0.1/2]); %3D dönüşüm matrisleri
coll2 = collisionCylinder(0.025,0.1); % cylinder: radius,length
coll2.Pose = trvec2tform([0 0 0.1/2]); %3D dönüşüm matrisleri
coll3 = collisionCylinder(0.025,0.1); % cylinder: radius,length
coll3.Pose = trvec2tform([0 0 0.1/2]); %3D dönüşüm matrisleri
collGripper = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
collGripper.Pose = trvec2tform([0 0 0.15/2]);%3D dönüşüm matrisleri
addCollision(rotatingBase,collBase)
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(gripper,collGripper)
jntBase = rigidBodyJoint("base_joint","revolute");
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jnt3 = rigidBodyJoint("jnt3","revolute");
jntgripper = rigidBodyJoint("gripper_joint","prismatic");
jnt1.JointAxis = [1 0 0]; % x-axis
jnt2.JointAxis = [1 0 0]; % x-axis
jnt3.JointAxis = [1 0 0]; % x-axis
jntgripper.JointAxis = [0 0 1]; % z-axis
setFixedTransform(jnt1,trvec2tform([0.015 0 0.04]))
setFixedTransform(jnt2,trvec2tform([-0.015 0 0.15/1.5]))
setFixedTransform(jnt3,trvec2tform([0.015 0 0.15/1.5]))
setFixedTransform(jntgripper,trvec2tform([0 0 0.15/2]))
bodies = {base,rotatingBase,arm1,arm2,arm3,gripper};
joints = {[],jntBase,jnt1,jnt2,jnt3,jntgripper};
figure("Name"," Robot","Visible","on")
for i = 2:length(bodies) % Skip base. Iterate through adding bodies and joints.
 bodies{i}.Joint = joints{i};
 addBody(robot,bodies{i},bodies{i-1}.Name)
 show(robot,"Collisions","on","Frames","off");
 drawnow;
end
% Belirli bir pozisyona hareket ettirme
jointPositions = [0, deg2rad(10), deg2rad(45), 0]; % Örnek: İlk eklem sabit olduğu için 0
animateRobot(robot, jointPositions);

function animateRobot(robot, jointPositions)
    % Robotu belirli eklemler pozisyonuna getiren fonksiyon
    for i = 1:length(jointPositions)
        config = robot.randomConfiguration; % Rastgele başlangıç konfigürasyonu alınır
        robot.show(config); % Belirli konfigürasyonu göstermek için kullanılır
        pause(2); % Hareketin gözlemlenebilmesi için kısa bir bekleme süresi
    end
end