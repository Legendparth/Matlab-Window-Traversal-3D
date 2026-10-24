
Va = 5;
x0 = 0;
y0 = 0;
z0 = 0;
chi0 = -pi/4;
gamma0 = 0;

wallTL = [200, 100, 150; 90, 300, 50; 200, 425, 100];
wallTR = [250, 100, 150; 140, 300, 50; 300, 425, 100];
wallBR = [250, 100, 100; 140, 300, 0; 300, 425, 50];
wallBL = [200, 100, 100; 90, 300, 0; 200, 425, 50];
offset = 30;
chiF = pi/2;
gammaF = 0;
r = 25;
num_waypoints = 200;
midPoint = (wallTL + wallTR + wallBR + wallBL) ./ 4;
goalPose = midPoint - [0, offset, 0]; goalPose(:, 4) = chiF;
[num, ~] = size(midPoint);
startPose = [x0, y0, z0; midPoint]; startPose(1, 4) = chi0; startPose(2:end, 4) = chiF;
l2 = zeros(0, num);sign1 = zeros(0, num);sign2 = zeros(0, num);dir1 = zeros(0, num); dir2 = zeros(0, num); gamma_const = zeros(0, num);
C1 = zeros(num, 2);C2 = zeros(num, 2);A = zeros(num, 2);B = zeros(num, 2);
ref_posesXY = zeros(num_waypoints+2, 3, num); ref_posesZ = zeros(1, num_waypoints + 2, num);

for i=1:num
    dubConn = dubinsConnection;
    dubConn.MinTurningRadius = r;
    [pathSeg, pathCost] = connect(dubConn, startPose(i, [1,2,4]), goalPose(i, [1,2,4]));
    
    motionTypes   = pathSeg{1}.MotionTypes;
    motionLengths = pathSeg{1}.MotionLengths;
    
    dir1 = motionTypes{1};
    dir2 = motionTypes{3};
    l1   = motionLengths(1);
    l2   = motionLengths(2);
    l3   = motionLengths(3);
    
    CLi = startPose(i, 1:2) + r * [-sin(startPose(i, 4)), cos(startPose(i, 4))];
    CRi = startPose(i, 1:2) + r * [sin(startPose(i, 4)), -cos(startPose(i, 4))];
    CLf = goalPose(i, 1:2) + r * [-sin(goalPose(i, 4)), cos(goalPose(i, 4))];
    CRf = goalPose(i, 1:2) + r * [sin(goalPose(i, 4)), -cos(goalPose(i, 4))];
    
    CLiCLf = norm(CLf - CLi);
    CLiCRf = norm(CRf - CLi);
    CRiCRf = norm(CRf - CRi);
    CRiCLf = norm(CLf - CRi);
    
    [val, idx] = min([CLiCLf, CLiCRf, CRiCRf, CRiCLf]);
    
    switch idx
    
        case 1
            C1(i, 1:2) = CLi;
            C2(i, 1:2) = CLf;
            dir1(i) = 'L'; dir2(i) = 'L';
            sign1(i) = 1;
            sign2(i) = 1;
            theta_c = atan2(C2(i, 2) - C1(i, 2), C2(i, 1) - C1(i, 1));
            theta_p = asin(2 * r / CLiCLf);
            arg = pi/2 - theta_c;
            A(i, 1:3) = [C1(i, 1:2) + r * [cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * (l1 / pathCost)];
            B(i, 1:3) = [C2(i, 1:2) + r * [cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * ((l1 + l2) / pathCost)];
        case 2
            C1(i, 1:2) = CLi;
            C2(i, 1:2) = CRf;
            dir1(i) = 'L'; dir2(i) = 'R';
            sign1(i) = 1;
            sign2(i) = -1;
            theta_c = atan2(C2(i, 2) - C1(i, 2), C2(i, 1) - C1(i, 1));
            theta_p = asin(2 * r / CLiCRf);
            alpha = pi/2 - theta_p;
            arg = theta_c - alpha;
            A(i, 1:3) = [C1(i, 1:2) + r * [cos(arg), sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * (l1 / pathCost)];
            B(i, 1:3) = [C2(i, 1:2) + r * [-cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * ((l1 + l2) / pathCost)];
        case 3
            C1(i, 1:2) = CRi;
            C2(i, 1:2) = CRf;
            dir1(i) = 'R'; dir2(i) = 'R';
            sign1(i) = -1;
            sign2(i) = -1;
            theta_c = atan2(C2(i, 2) - C1(i, 2), C2(i, 1) - C1(i, 1));
            theta_p = asin(2 * r / CRiCRf);
            arg = theta_c - pi/2;
            A(i, 1:3) = [C1(i, 1:2) + r * [-cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * (l1 / pathCost)];
            B(i, 1:3) = [C2(i, 1:2) + r * [-cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * ((l1 + l2) / pathCost)];
        case 4
            C1(i, 1:2) = CRi;
            C2(i, 1:2) = CLf;
            dir1(i) = 'R'; dir2(i) = 'L';
            sign1(i) = -1;
            sign2(i) = 1;
            theta_c = atan2(C2(i, 2) - C1(i, 2), C2(i, 1) - C1(i, 1));
            theta_p = asin(2 * r / CRiCLf);
            alpha = pi/2 - theta_p;
            arg = theta_c + alpha;
            A(i, 1:3) = [C1(i, 1:2) + r * [cos(arg), sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * (l1 / pathCost)];
            B(i, 1:3) = [C2(i, 1:2) + r * [-cos(arg), -sin(arg)], startPose(i, 3) + (goalPose(i, 3) - startPose(i, 3)) * ((l1 + l2) / pathCost)];
    end
    
    gamma_const(i) = asin((B(i, 3) - A(i, 3)) / l2);
    length = sum(pathSeg{1}.MotionLengths);
    
    ref_posesXY(:, :, i) = interpolate(pathSeg{1}, linspace(0, length, num_waypoints));
    ref_posesZ(:, :, i) = linspace(startPose(i, 3), goalPose(i, 3), num_waypoints+2);
end

