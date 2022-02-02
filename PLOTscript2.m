%%
% https://stackoverflow.com/questions/40836018/rotation-matrix-between-two-camera-system
% R01 is the rotation from reference system 1 (first camera) to system 0 (world)
% 0 world 1 cam1 2 cam2 inv(R02)*R01 =  R20*R01 = R21

% https://en.wikipedia.org/wiki/Essential_matrix
% https://en.wikipedia.org/wiki/Camera_resectioning
% https://math.stackexchange.com/questions/709622/relative-camera-matrix-pose-from-global-camera-matrixes
clear;
% close all
List=round(linspace(1,648,30));
load IMUData
imucamlog2=IMIU((length(IMIU)-648+1):length(IMIU),12:15);
% imucamlog2=imucamlog2(List,:);
imucamlog2=medfilt1(imucamlog2);

load('orient.mat')

x=linspace(0,1,100);
X(1:100,1)=x;X(:,2:3)=0;
X(101:200,2)=x;
X(201:300,3)=x;

% figure(1)
load('OutputFiles\GyroCal2');

for Sk=1:length(imucamlog2)-1
    
subplot(221)
imshow(imresize(imread(['UndistortedImageDirectory\im',num2str(Sk+1+23,'%03d'),'.png']),0.25))
% Rc=rotationVectorToMatrix(rvecs(ii+1,:))*rotationVectorToMatrix(rvecs(ii,:))';
A1=-rotationVectorToMatrix(rvecs(1+22,:));
A2=-rotationVectorToMatrix(rvecs(Sk+1+22,:));
Rc=(A2*A1');
datac1=Rc*X';
subplot(222)
pcshow(datac1',ceil(X),'markersize',20)
xlabel('x');ylabel('y');zlabel('z');
xlim([-1,1]);ylim([-1,1]);zlim([-1,1])
title('CAM')
% Rg=quat2rotm(imucamlog2(ii+1,:))*quat2rotm(imucamlog2(ii,:))';

R1=-quat2rotm(imucamlog2(1,:));
R2=-quat2rotm(imucamlog2(Sk+1,:));
Rg=(R2*R1');

datac2=Rg*X';
subplot(223)
pcshow(datac2',ceil(X),'markersize',20)
xlabel('x');ylabel('y');zlabel('z');
xlim([-1,1]);ylim([-1,1]);zlim([-1,1])
title('IMU (Raw)')
eul = rotm2eul(Z'*Rg*Z);eul(2:3)=0;
rotm = eul2rotm(eul);
% datac2=Z'*Rg*Z*X';
datac2=rotm*X';

subplot(224)
pcshow(datac2',ceil(X),'markersize',20)
xlabel('x');ylabel('y');zlabel('z');
xlim([-1,1]);ylim([-1,1]);zlim([-1,1])
title('IMU (corrected)')
M(Sk) = getframe(gcf);
% pause(1/15);
end

%%
% csvwrite('myFile.csv',M)
% figure
% movie(M,5)