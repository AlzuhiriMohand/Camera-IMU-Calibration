%%
%%-https://math.stackexchange.com/questions/3028145/how-to-solve-ax-xb-for-x-matrix
clear;%clc
load orient;
% List=[2,64,104,183,253,320,365,415,453,520,569,615,668,709,747,834,879,940,1007,1072,1133,1271,1306,1353,1411,1460,1515,1595,1631,1668];
% List=List-1;
% List=round(linspace(1,648,30));
load IMIU
IMIU=medfilt1(IMIU,6);
imucamlog2=IMIU((length(IMIU)-648+1):length(IMIU),12:15);
% imucamlog2=imucamlog2(List,:);
plot((0:length(imucamlog2)-1)/15,imucamlog2)
xlabel('Time (s)')
ylabel('Amplitude (a.u)')
set(gca,'fontsize',18)
legend('a','b','c','d')
%%
global Nof At Bt
Nof=647-22;
At=zeros(4,4,Nof);
Bt=At;
for Sk=1:Nof
    %-------------------Roll,  Pitch, and Yaw......X Y Z
    %--our data   Head (Yaw), Roll,  and Pitch.....Z X Y
    %--We arrange it to have                       Z Y X
    R1=quat2rotm(imucamlog2(1,:));
    R2=quat2rotm(imucamlog2(Sk+1,:));
    B=(R2*R1');
    Bt(1:3,1:3,Sk)=B;Bt(4,4,Sk)=1;
    
    A1=-rotationVectorToMatrix(rvecs(1+22,:));
    A2=-rotationVectorToMatrix(rvecs(Sk+1+22,:));
    A=(A2*A1');
    At(1:3,1:3,Sk)=A';At(4,4,Sk)=1;
end
%%
% clc
% tic

Z=handEye(Bt,At);
% toc
Z=real(Z(1:3,1:3));
%%
fitLineFcn = @(y)handEye(Bt(:,:,y),At(:,:,y));
evalLineFcn = @(model,y) tester(model,y);

sampleSize = 30; % number of points to sample per trial
maxDistance = 0.2;0.005; % max allowable distance for inliers
y=1:length(At);
[modelRANSAC, inlierIdx] = ransac(y',fitLineFcn,evalLineFcn,sampleSize,maxDistance);
At=At(:,:,inlierIdx);
Bt=Bt(:,:,inlierIdx);
Z=handEye(Bt,At);
Z=Z(1:3,1:3);
% hold on
% pcshow((Z*r')','b')%
%%
% load Z.mat
clc
Err=0;
for i=1:length(At)
B=Bt(1:3,1:3,i);
A=At(1:3,1:3,i);
Errt=A-(Z'*B*Z);
Err(i)=sum(abs(Errt(:)))/9;
end
(mean(Err))

function Err=tester(model,y)
global Nof At Bt
Z=model(1:3,1:3);
for i=1:Nof
B=Bt(1:3,1:3,i);
A=At(1:3,1:3,i);
Errt=rotm2quat(A)-rotm2quat((Z'*B*Z));
Err(i,1)=sum(abs(Errt(:)))/4;
end
end
% tst=Z'*B*Z
