clc;clear
width=1664;
height=1232;
R=1.5;i=0;I=1;
Name='imu_cam_cal_final_cal';
load('Calresults.mat');
Knew(1,1)=470;Knew(2,2)=Knew(1,1);Knew(1,2)=0;
List=round(linspace(1,648,30));
fileID = fopen([Name,'.dat']);
while true
    OneByte = fread(fileID,width*height*R,'uint8',0);
    if isempty(OneByte)
        disp('Total frames: ')
        disp(i)
        disp('End of stream')
        break
    else
        i=i+1;
        A=double(OneByte);
        Y=reshape(OneByte(1:width*height),[1664,1232])';
        U=reshape(OneByte(width*height+1:1.25*width*height),[width/2,height/2])';
        V=reshape(OneByte(1.25*width*height+1:1.5*width*height),[width/2,height/2])';
        im(:,:,1)=Y;
        im(:,:,2)=imresize(U,2);
        im(:,:,3)=imresize(V,2);
        im=ycbcr2rgb(uint8(im));
        imu=cv.fisheyeUndistortImage(im,K,D,'NewCameraMatrix',Knew);
%         if i==List(I)
            imwrite(imu,['UndistortedImageDirectory\im',num2str(I,'%03d'),'.png']);
            I=I+1
%         end
    end
end
fclose(fileID);
