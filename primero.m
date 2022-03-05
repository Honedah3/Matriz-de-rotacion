e = [-1;1];
figure(1);
plot3(e,[0;0],[0;0],'r','lineWidth',2);
hold on;
plot3([0;0],e,[0;0],'g','lineWidth',2);
hold on;
plot3([0;0],[0;0],e,'b','lineWidth',2);
hold on;
grid on;
xlabel('Eje x','Color','r');
ylabel('Eje y','Color','g');
zlabel('Eje z','Color','b');
hold on;
x = [1;0;0];
y = [0;1;0];
z = [0;0;1];
 view(3);
 
%Definir valores animacion
frames = 1000;
t_pausa = 0.001
t_pausaInicio = 0.1;
t_pausaFin = 10000;

alpha = 90;
beta = 90;
theta = pi/9;
%% Rotación con respecto al eje x:
for i = 1:frames
    %Condicion para que se pause un poco al inicio
    if(i == 1)
       pause(t_pausaInicio) 
    end 
%Convercion para avanzar un poco cada ciclo
    div = (i/frames);
    thetan = theta*div;
    
th = thetan;
 Rx = [1 0 0;0 cos(th) sin(th);0 -sin(th) cos(th)]; %Es la matriz de rotacion en el eje X en el sentido opuesto a las agujas del reloj del espacio 3D
 x = Rx*x;
 quiver3(0,0,0,x(1),x(2),x(3),'r');
 hold on;
 y = Rx*y;
 quiver3(0,0,0,y(1),y(2),y(3),'g');
 hold on;
 z = Rx*z;
 quiver3(0,0,0,z(1),z(2),z(3),'b');
 hold on;
 t = 0:.1:2*pi;
 p1 = 0.35*cos(t);
 p2 = 0.35*sin(t);
 l = length(t);
 for i = 1:l
     p3(i)=1;
 end
 p1 = p1*cos(th);
 p2 = p2*cos(th);
 p3 = p3*cos(th);
 plot3(p3,p1,p2);
 hold on;
 
 pause(t_pausa); %Pausa
 if (i == 100)
     pause(t_pausaFin) 
    end 
end
