% Units are centimetres
% Obstacle 1: Cylinder
obs{1}.R = 100;
obs{1}.c = [6.2;0]*100;
obs{1}.rho0 = 150;
obs{1}.h = 572;
obs{1}.type = 'cyl';

obs{2}.type = 'plane';
obs{2}.rho0 = 250;

obs{3}.R = 100;
obs{3}.c = [6.2;-4.4]*100;
obs{3}.rho0 = 150;
obs{3}.h = 572;
obs{3}.type = 'cyl';