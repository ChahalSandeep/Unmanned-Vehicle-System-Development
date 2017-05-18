function []=myAnimation(xActEst,yActEst,thetaAct,X_array,Y_array,rp1,rp2)
ugvWidth= 2*0.3556;
ugvLength=2*0.4635;
ugvShape = [-ugvLength/2 -ugvLength/2 ugvLength/2 ugvLength/2;
            ugvWidth/2 -ugvWidth/2 -ugvWidth/2 ugvWidth/2];
        
figure('units','normalized','position',[.1 .1 .75 .75])
plot(yActEst.signals.values(:,1),xActEst.signals.values(:,1));
hold on
plot(Y_array,X_array,'*r')
vAxis = axis;
hold off
plot(Y_array,X_array,'*r')
hold on;
grid
xlabel('y [m]')
ylabel('x [m]')
for ii = 1:length(X_array)
    my_circle(rp2,Y_array(ii),X_array(ii),1,1,'-r',1)
    my_circle(rp1,Y_array(ii),X_array(ii),1,1,'--b',1)
end
axis image
axis([vAxis(1)-2*max(rp1,rp2) vAxis(2)+2*max(rp1,rp2) vAxis(3)-2*max(rp1,rp2) vAxis(4)+2*max(rp1,rp2)])
ugvHandle = fill(ugvShape(2,:),ugvShape(1,:),'b');
tailHandle = plot(yActEst.signals.values(1,1),xActEst.signals.values(1,1));
tailL = 150;
for iii=1:length(xActEst.signals.values(:,1))
 delete(ugvHandle)
 delete(tailHandle)
 ugvShapeRot = rotMat2D((pi/180)*thetaAct.signals.values(iii,1))*ugvShape;
 ugvShapeMov = [ugvShapeRot(1,:)+xActEst.signals.values(iii,1);ugvShapeRot(2,:)+yActEst.signals.values(iii,1)];
 
 ugvHandle = fill(ugvShapeMov(2,:),ugvShapeMov(1,:),'b');
 tailHandle = plot(yActEst.signals.values([iii:-1:max(iii-tailL,1)],1),xActEst.signals.values([iii:-1:max(iii-tailL,1)],1),':b','Linewidth',2);

 pause(0.001)
 
end

function R = rotMat2D(angle)
R = [cos(angle) -sin(angle);
     sin(angle)  cos(angle)];

 function my_circle(r,x_c,y_c,x_scaling,h_scaling,lt,lw)
%
% draws a circle with radius r, centered at (x_c,y_c), linetype lt, and line
% width lw
%
angle=-pi:.01:pi;
for i = 1:length(angle)
    R=[x_scaling*cos(angle(i)),-x_scaling*sin(angle(i));...
        h_scaling*sin(angle(i)),h_scaling*cos(angle(i))];    
    temp=R*[r;0];
    x_circle(i)=temp(1);
    y_circle(i)=temp(2);
end
X_circle=x_c+x_circle;
Y_circle=y_c+y_circle;
plot(X_circle,Y_circle,lt,'LineWidth',lw)
