function y = psobj(x)
global A
global B
global C
global D
A = [0  0  0   1   0   0;
     0  0  0   0   1   0;
     0  0  0   0   0   1;
     0  0  0   0   0   0;
     0  0  0   0   0   0;
     0  0  0   0   0   0];

B = [0 0 0  0;
     0 0 0  0;
     0 0 0  0;
     0 x(1) 0 -x(1) ;
     x(2) 0 -x(2)  0;
     -x(3) x(3) x(3)  x(3)];

C =[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1]; 

D =zeros(6,4);
try
sim('model_drone_calcul');
load erreur.mat
y1 = sum(erreur.Data.*erreur.Data);
catch
   y1 = 1000000000; 
end
y = sum(y1);