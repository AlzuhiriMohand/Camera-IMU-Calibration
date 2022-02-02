%%
%%-https://math.stackexchange.com/questions/3028145/how-to-solve-ax-xb-for-x-matrix
clear;%clc
load orient;
load IMUData
IMIU=medfilt1(IMIU,6);  %---Cleaning of outliers
imucamlog2=IMIU((length(IMIU)-648+1):length(IMIU),12:15);  %---Synch with the camera data
plot((0:length(imucamlog2)-1)/15,imucamlog2)
xlabel('Time (s)')
ylabel('Amplitude (a.u)')
set(gca,'fontsize',18)
legend('a','b','c','d')
%%
Nof=647-22;
At=zeros(4,4,Nof);
Bt=At;
for Sk=1:Nof
    %-------------------Roll,  Pitch, and Yaw......X Y Z
    %--our data   Head (Yaw), Roll,  and Pitch.....Z X Y
    %--We arrange it to have                       Z Y X
    R1=quat2rotm(imucamlog2(Sk,:));
    R2=quat2rotm(imucamlog2(Sk+1,:));
    B=(R2*R1');
    Bt(1:3,1:3,Sk)=B;Bt(4,4,Sk)=1;
    
    A1=-rotationVectorToMatrix(rvecs(Sk+22,:));
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
save('OutputFiles\GyroCal2','Z')

% hold on
% pcshow((Z*r')','b')%
%%
% load Z6.mat
clc
Err=0;
for i=1:Nof
B=Bt(1:3,1:3,i);
A=At(1:3,1:3,i);
Errt=A-(Z'*B*Z);
Err(i)=sum(abs(Errt(:)))/9;
end
sqrt(mean(Err))

% tst=Z'*B*Z
