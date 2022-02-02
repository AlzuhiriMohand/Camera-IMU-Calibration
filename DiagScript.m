% %%
% v = VideoWriter('newfile.avi','Motion JPEG AVI');
% v.Quality = 90;
% open(v)
% for i=1:624
% writeVideo(v,M(i).cdata)
% end
% close(v)
%%
load('OutputFiles\GyroCal2');
for i=1:600
A1=-rotationVectorToMatrix(rvecs(1+22,:));
A2=-rotationVectorToMatrix(rvecs(i+1+22,:));
Rc=(A2*A1');
% quat1(i,:) = rotm2quat(Rc);
quat1(i,:) = rotm2eul(Rc);

R1=-quat2rotm(imucamlog2(1,:));
R2=-quat2rotm(imucamlog2(i+1,:));
Rg=(R2*R1');
Rg=Z'*Rg*Z;
% quat2(i,:) = rotm2quat(Rg);
quat2(i,:) = rotm2eul(Rg);


end
figure
% subplot(311)
ind=2;
plot((1:length(quat2))/15,quat1(:,ind));
hold on
plot((1:length(quat2))/15,quat2(:,ind))
hold off
xlabel('Time (s)')
ylabel('Angle (Rad)')
ylim([-1.8,1.8])
set(gca,'fontsize',14)
legend('IMU','Camera')
RMSE=sqrt(mean((quat1(:,ind)-quat2(:,ind)).^2))
title(['RMSE= ',num2str(RMSE),' Rad'])

figure
% subplot(311)

plot((1:length(quat2))/15,abs(quat1(:,ind)-quat2(:,ind)));
hold on
plot((1:length(quat2))/15,ones(1,length(quat2))*max(abs(quat1(:,ind)-quat2(:,ind))));

xlabel('Time (s)')
ylabel('Absolute value of error (Rad)')
ylim([0,0.2])
set(gca,'fontsize',14)


% title(['Absolute value of error'])