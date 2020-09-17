close all
%clear all
format long

Roulis = 1;
Tangage = 1;
Lacet = 1;
Pid = 1;
Manette = 1;
Frequence = 1;
AngleVitesse = 1;
Altitude = 1;
Commande = 1;
Spectrale = 0;

cd 'C:\Users\Guitou\Desktop\drone_ws\src\drone\src\Data'
ANGLES =   load( "Angles.txt");
ALTITUDE =   load( "Altitude.txt");
ACCELMAG =   load( "AccelMag.txt");

PID =   load( "PID.txt");
PID_vit =   load( "PID_Vit.txt");
COMMANDE_moteur =   load( "COMMANDE_moteur.txt");
MANETTE =   load( "Manette.txt");
DATA =   load( "Data.txt");
PID_decompose_altitude =   load( "PID_decompose_altitude.txt");
PID_decompose_lacet =   load( "PID_decompose_lacet.txt");
PID_decompose_roulis =   load( "PID_decompose_roulis.txt");
PID_decompose_tangage =   load( "PID_decompose_tangage.txt");
PID_vit_decompose_altitude =   load( "PID_vit_decompose_altitude.txt");
PID_vit_decompose_lacet =   load( "PID_vit_decompose_lacet.txt");
PID_vit_decompose_roulis =   load( "PID_vit_decompose_roulis.txt");
PID_vit_decompose_tangage =   load( "PID_vit_decompose_tangage.txt");



[a,b] = min(DATA(:,2))
D = sort(DATA);
taille  =length(ANGLES)-10;
fin = taille;
debut = 1;
%taille = 10500;

if Frequence
subplot(1,2,1)
plot(DATA(:,2))
title('frequence servos')

subplot(1,2,2)
plot(DATA(:,3))
title('frequence capteur')
end

if AngleVitesse
figure
subplot(3,2,1)
plot(ANGLES(debut:fin,1))
title('Angles Tangage')
subplot(3,2,3)
plot(ANGLES(debut:fin,2))
title('Angles Roulis')
subplot(3,2,5)
plot(ANGLES(debut:fin,3))
title('Angles Lacet')
subplot(3,2,2)
plot(ANGLES(debut:fin,4))
title('Angles vitesse Tangage')
subplot(3,2,4)
plot(ANGLES(debut:fin,5))
title('Angles vitesse Roulis')
subplot(3,2,6)
plot(ANGLES(debut:fin,6))
title('Angles vitesse Lacet')
end


figure
subplot(3,3,1)
plot(ANGLES(debut:fin,4))
title('Gx')
subplot(3,3,2)
plot(ANGLES(debut:fin,5))
title('Gy')
subplot(3,3,3)
plot(ANGLES(debut:fin,6))
title('Gz')
subplot(3,3,4)
plot(ACCELMAG(debut:fin,1))
title('AX')
subplot(3,3,5)
plot(ACCELMAG(debut:fin,2))
title('AY')
subplot(3,3,6)
plot(ACCELMAG(debut:fin,3))
title('AZ')
subplot(3,3,7)
plot(ACCELMAG(debut:fin,4))
title('MX')
subplot(3,3,8)
plot(ACCELMAG(debut:fin,5))
title('MY')
subplot(3,3,9)
plot(ACCELMAG(debut:fin,6))
title('MZ')

if Pid
figure
subplot(3,3,1)
plot(PID(debut:fin,1))
title('kp roulis/tangante')
subplot(3,3,2)
plot(PID(debut:fin,2))
title('ki roulis/tangante')
subplot(3,3,3)
plot(PID(debut:fin,3))
title('kd roulis/tangante')
subplot(3,3,4)
plot(PID(debut:fin,4))
title('kp lacet')
subplot(3,3,5)
plot(PID(debut:fin,5))
title('ki lacet')
subplot(3,3,6)
plot(PID(debut:fin,6))
title('kd lacet')
subplot(3,3,7)
plot(PID(debut:fin,7))
title('kp altitude')
subplot(3,3,8)
plot(PID(debut:fin,8))
title('ki altitude')
subplot(3,3,9)
plot(PID(debut:fin,9))
title('kd altitude')


figure
subplot(3,3,1)
plot(PID_vit(debut:fin,1))
title('kp vit roulis/tangante')
subplot(3,3,2)
plot(PID_vit(debut:fin,2))
title('ki vit roulis/tangante')
subplot(3,3,3)
plot(PID_vit(debut:fin,3))
title('kd vit roulis/tangante')
subplot(3,3,4)
plot(PID_vit(debut:fin,4))
title('kp vit lacet')
subplot(3,3,5)
plot(PID_vit(debut:fin,5))
title('ki vit lacet')
subplot(3,3,6)
plot(PID_vit(debut:fin,6))
title('kd vit lacet')
subplot(3,3,7)
plot(PID_vit(debut:fin,7))
title('kp vit altitude')
subplot(3,3,8)
plot(PID_vit(debut:fin,8))
title('ki vit altitude')
subplot(3,3,9)
plot(PID_vit(debut:fin,9))
title('kd vit altitude')
end

if Manette
figure
subplot(5,1,1)
plot(MANETTE(debut:fin,1))
title('CONSIGNE Gaz')
subplot(5,1,2)
plot(MANETTE(debut:fin,2))
title('CONSIGNE Tangage')
subplot(5,1,3)
plot(MANETTE(debut:fin,3))
title('CONSIGNE Roulis')
subplot(5,1,4)
plot(MANETTE(debut:fin,4))
title('CONSIGNE Lacet')
subplot(5,1,5)
plot(MANETTE(debut:fin,5))
title('CONSIGNE Altitude')
end

if Altitude
figure
subplot(5,1,1)
plot(ALTITUDE(debut:fin,1))
title('Altitude barometre')
subplot(5,1,2)
plot(ALTITUDE(debut:fin,2))
title('Altitude ultrason')
subplot(5,1,3)
plot(ALTITUDE(debut:fin,3))
title('Altitude utilis� dans calcul')
subplot(5,1,4)
plot(ALTITUDE(debut:fin,4))
title('Altitude vitesse barometre')
subplot(5,1,5)
plot(ALTITUDE(debut:fin,5))
title('Altitude  vitesse ultrason')
end

if Commande
figure
subplot(4,1,1)
plot(COMMANDE_moteur(debut:fin,1))
title('commande devant droit')
subplot(4,1,2)
plot(COMMANDE_moteur(debut:fin,2))
title('commande devant_gauche')
subplot(4,1,3)
plot(COMMANDE_moteur(debut:fin,3))
title('commande derierre droit')
subplot(4,1,4)
plot(COMMANDE_moteur(debut:fin,4))
title('commande derierre_gauche')
end

if Pid
figure
subplot(4,2,1)
plot(PID_decompose_tangage(debut:fin,1))
title('Kp tangage')
subplot(4,2,3)
plot(PID_decompose_tangage(debut:fin,2))
title('Ki tangage')
subplot(4,2,5)
plot(PID_decompose_tangage(debut:fin,3))
title('Kd tangage')
subplot(4,2,7)
plot(PID_decompose_tangage(debut:fin,1)+PID_decompose_tangage(debut:fin,2)+PID_decompose_tangage(debut:fin,3))
title('Commande tangage')

subplot(4,2,2)
plot(PID_vit_decompose_tangage(debut:fin,1))
title('Kp vit tangage')
subplot(4,2,4)
plot(PID_vit_decompose_tangage(debut:fin,2))
title('Ki vit tangage')
subplot(4,2,6)
plot(PID_vit_decompose_tangage(debut:fin,3))
title('Kd vit tangage')
subplot(4,2,8)
plot(PID_vit_decompose_tangage(debut:fin,1)+PID_vit_decompose_tangage(debut:fin,2)+PID_vit_decompose_tangage(debut:fin,3))
title('Commande vit tangage')


figure
subplot(4,2,1)
plot(PID_decompose_roulis(debut:fin,1))
title('Kp roulis')
subplot(4,2,3)
plot(PID_decompose_roulis(debut:fin,2))
title('Ki roulis')
subplot(4,2,5)
plot(PID_decompose_roulis(debut:fin,3))
title('Kd roulis')
subplot(4,2,7)
plot(PID_decompose_roulis(debut:fin,1)+PID_decompose_roulis(debut:fin,2)+PID_decompose_roulis(debut:fin,3))
title('Commande roulis')

subplot(4,2,2)
plot(PID_vit_decompose_roulis(debut:fin,1))
title('Kp vit roulis')
subplot(4,2,4)
plot(PID_vit_decompose_roulis(debut:fin,2))
title('Ki vit roulis')
subplot(4,2,6)
plot(PID_vit_decompose_roulis(debut:fin,3))
title('Kd vit roulis')
subplot(4,2,8)
plot(PID_vit_decompose_roulis(debut:fin,1)+PID_vit_decompose_roulis(debut:fin,2)+PID_vit_decompose_roulis(debut:fin,3))
title('Commande vit roulis')


figure
subplot(4,2,1)
plot(PID_decompose_lacet(debut:fin,1))
title('Kp lacet')
subplot(4,2,3)
plot(PID_decompose_lacet(debut:fin,2))
title('Ki lacet')
subplot(4,2,5)
plot(PID_decompose_lacet(debut:fin,3))
title('Kd lacet')
subplot(4,2,7)
plot(PID_decompose_lacet(debut:fin,1)+PID_decompose_lacet(debut:fin,2)+PID_decompose_lacet(debut:fin,3))
title('Commande lacet')

subplot(4,2,2)
plot(PID_vit_decompose_lacet(debut:fin,1))
title('Kp vit lacet')
subplot(4,2,4)
plot(PID_vit_decompose_lacet(debut:fin,2))
title('Ki vit lacet')
subplot(4,2,6)
plot(PID_vit_decompose_lacet(debut:fin,3))
title('Kd vit lacet')
subplot(4,2,8)
plot(PID_vit_decompose_lacet(debut:fin,1)+PID_vit_decompose_lacet(debut:fin,2)+PID_vit_decompose_lacet(debut:fin,3))
title('Commande vit lacet')


figure
subplot(4,2,1)
plot(PID_decompose_altitude(debut:fin,1))
title('Kp altitude')
subplot(4,2,3)
plot(PID_decompose_altitude(debut:fin,2))
title('Ki altitude')
subplot(4,2,5)
plot(PID_decompose_altitude(debut:fin,3))
title('Kd altitude')
subplot(4,2,7)
plot(PID_decompose_altitude(debut:fin,1)+PID_decompose_altitude(debut:fin,2)+PID_decompose_altitude(debut:fin,3))
title('Commande altitude')

subplot(4,2,2)
plot(PID_vit_decompose_altitude(debut:fin,1))
title('Kp vit altitude')
subplot(4,2,4)
plot(PID_vit_decompose_altitude(debut:fin,2))
title('Ki vit altitude')
subplot(4,2,6)
plot(PID_vit_decompose_altitude(debut:fin,3))
title('Kd vit altitude')
subplot(4,2,8)
plot(PID_vit_decompose_altitude(debut:fin,1)+PID_vit_decompose_altitude(debut:fin,2)+PID_vit_decompose_altitude(debut:fin,3))
title('Commande vit altitude')
end

if  Spectrale
taille  =length(ANGLES)-10;
ANGLES =   load( "Angles.txt");
Fs  = 1000;
nfft = taille;
Y =fft(ANGLES(:,4),nfft);
Y = Y(1:nfft/2);
my = abs(Y);
f = (0:nfft/2-1)*Fs/nfft;
figure
plot(f,my);
title('Power Spectrum of TANGANTE');
xlabel('Frequency (Hz)');
ylabel('Power');

taille  =length(ANGLES)-10;
ANGLES =   load( "Angles.txt");
Fs  = 1000;
nfft = taille;
Y =fft(ANGLES(:,5),nfft);
Y = Y(1:nfft/2);
my = abs(Y);
f = (0:nfft/2-1)*Fs/nfft;
figure
plot(f,my);
title('Power Spectrum of ROULIS');
xlabel('Frequency (Hz)');
ylabel('Power');

end

%%
%%%%%%%%%%%%
%%%%Simulink%%%%
close all
taille = length(DATA);
time(1,1)=DATA(1,1);
for i=2:taille
    time(i,1) = time(i-1,1)+DATA(i,1);
end

tangage = [time,-ANGLES(:,1)];
roulis = [time,-ANGLES(:,2)];
lacet = [time,-ANGLES(:,3)];
tangage_vit = [time,-ANGLES(:,4)];
roulis_vit =[time,-ANGLES(:,5)];
lacet_vit = [time,-ANGLES(:,6)];
gaz=[time,MANETTE(:,1)];
commande_moteur_input  = [time,COMMANDE_moteur];
PID_complet_tangage  = [time,PID_decompose_tangage(:,1)+PID_decompose_tangage(:,2)+PID_decompose_tangage(:,3)];
%%

load SteamEng
sortie = ANGLES;
entree = COMMANDE_moteur;
taille = min(length(sortie),length(COMMANDE_moteur));
steam = iddata(sortie(debut:fin,:),COMMANDE_moteur(debut:fin,:),0.004);
%%
close all
taille = length(DATA);
time(1,1)=DATA(1,1);
for i=2:taille
    time(i,1) = time(i-1,1)+DATA(i,1);
end

tangage = [time,-ANGLES(:,1)];
roulis = [time,-ANGLES(:,2)];
lacet = [time,-ANGLES(:,3)];
tangage_vit = [time,-ANGLES(:,4)];
roulis_vit =[time,-ANGLES(:,5)];
lacet_vit = [time,-ANGLES(:,6)];
gaz=[time,MANETTE(:,1)];
commande_moteur_input  = [time,COMMANDE_moteur];
PID_complet_tangage  = [time,PID_decompose_tangage(:,1)+PID_decompose_tangage(:,2)+PID_decompose_tangage(:,3)];

global A
global B
global C
global D
fun = @psobj;
x0 = rand(1,4);
x = patternsearch(fun,x0);
sim('model_drone_calcul');
%%
close all
taille = length(DATA);
time(1,1)=DATA(1,1);
for i=2:taille
    time(i,1) = time(i-1,1)+DATA(i,1);
end

tangage = [time,-ANGLES(:,1)];
roulis = [time,-ANGLES(:,2)];
lacet = [time,-ANGLES(:,3)];
tangage_vit = [time,-ANGLES(:,4)];
roulis_vit =[time,-ANGLES(:,5)];
lacet_vit = [time,-ANGLES(:,6)];
gaz=[time,MANETTE(:,1)];
commande_moteur_input  = [time,COMMANDE_moteur];
PID_complet_tangage  = [time,PID_decompose_tangage(:,1)+PID_decompose_tangage(:,2)+PID_decompose_tangage(:,3)];
PID_complet_roulis  = [time,PID_decompose_roulis(:,1)+PID_decompose_roulis(:,2)+PID_decompose_roulis(:,3)];
PID_complet_lacet  = [time,PID_decompose_lacet(:,1)+PID_decompose_lacet(:,2)+PID_decompose_lacet(:,3)];

global A
global B
global C
global D
fun = @psobj;
x0 = rand(1,3);
% 1. Establish bounds for variables
limit = [0 20];
for i=1:3
    bounds(i,:) = limit;
end

% 2. Send options to Direct
%    We tell DIRECT that the globalmin = 3
%    It will stop within 0.01% of solution
options.testflag  = 1; options.globalmin = 0; options.showits   = 1;
options.tol       = 0.01;

% 2a. NEW!
% Pass Function as part of a Matlab Structure
Problem.f = 'psobj';

% 3. Call DIRECT
[fmin,xmin,hist] = Direct(Problem,bounds,options);

% 4. Plot iteration statistics
plot(hist(:,2),hist(:,3))
xlabel('Fcn Evals');
ylabel('f_{min}');
title('Iteration Statistics for GP test Function');


for i=2:taille
    vitx(i-1) = (ANGLES(i,1) - ANGLES(i-1,1))/(1/490);
end
vitx = vitx';
diff = ANGLES(debut:fin-1,4) - vitx;
figure
plot(diff)

figure
plot(vitx)
hold on
plot(ANGLES(:,4))
legend()


close all

%%
alpha = 0.01
taille  =length(ANGLES)-10;
ANGLES =   load( "Angles.txt");
Fs  = 1000;
nfft = taille;
Y =fft(ANGLES(:,5),nfft);
Y = Y(1:nfft/2);
my = abs(Y);
f = (0:nfft/2-1)*Fs/nfft;
figure
plot(f,my);
title('Power Spectrum of a Sine Wave');
xlabel('Frequency (Hz)');
ylabel('Power');

for i = 1:1
    Vitesse_angles_filtre(1,1)=ANGLES(1,4);
    for j=2:taille
        Vitesse_angles_filtre(j,i)=ANGLES(j,4)*(alpha)+Vitesse_angles_filtre(j-1,1)*(1-alpha);
    end
    figure
    subplot(2,1,1)
    plot(ANGLES(:,4))
    subplot(2,1,2)
    plot(Vitesse_angles_filtre(:,i))
    nfft = taille;
    Y =fft(Vitesse_angles_filtre(:,i),nfft);
    Y = Y(1:nfft/2);
    my = abs(Y);
    f = (0:nfft/2-1)*Fs/nfft;
    figure
    plot(f,my);
    title('Power Spectrum of a Sine Wave');
    xlabel('Frequency (Hz)');
    ylabel('Power');
end
%%
ANGLES =   load( "AccelMag.txt");
Fs  = 4000;
nfft = taille;
Y =fft(ANGLES(:,1),nfft);
Y = Y(1:nfft/2);
my = abs(Y);
f = (0:nfft/2-1)*Fs/nfft;
figure
plot(f,my);
title('Power Spectrum of a Sine Wave');
xlabel('Frequency (Hz)');
ylabel('Power');

for i = 1:1
    Vitesse_angles_filtre(1,1)=ANGLES(1,1);
    for j=2:taille
        Vitesse_angles_filtre(j,i)=ANGLES(j,1)*(0.1)+Vitesse_angles_filtre(j-1,1)*(1-0.1);
    end
    figure
    subplot(2,1,1)
    plot(ANGLES(:,1))
    subplot(2,1,2)
    plot(Vitesse_angles_filtre(:,i))
    Fs  = 4000;
    nfft = taille;
    Y =fft(Vitesse_angles_filtre(:,i),nfft);
    Y = Y(1:nfft/2);
    my = abs(Y);
    f = (0:nfft/2-1)*Fs/nfft;
    figure
    plot(f,my);
    title('Power Spectrum of a Sine Wave');
    xlabel('Frequency (Hz)');
    ylabel('Power');
end
%%
clear all
close all
data = ones(1,10000000);
counter_loop = 1;
figure
%fgh = figure(); % create a figure
%axh = axes('Parent',fgh); % create axes
%plothandle = plot(NaN,NaN);
h = animatedline('MaximumNumPoints',150);
ylim([-50 50]);
%xlim([0 25]);

setappdata(gca,'LegendColorbarManualSpace',1);
setappdata(gca,'LegendColorbarReclaimSpace',1);
drawnow;
while(1)
    %t = tcpip('192.168.0.132', 22132, 'NetworkRole', 'server');
    t = tcpip('0.0.0.0', 22132, 'NetworkRole', 'server');
    fopen(t);
    
    while(1)
        if (t.BytesAvailable>0)
            %t.BytesAvailable
            
            raw_data = fread(t, 4);
            data(counter_loop) = typecast(uint8(raw_data), 'single');
            %disp(data);
            %plot(counter_loop,data(counter_loop),'b+');
            %axes(plothandle.axes1);
            %set(plothandle,'XData',1:counter_loop,'YData',data(1:counter_loop));
            %plothandle.XData(end+1) = counter_loop;
            %plothandle.YData(end+1) = data(counter_loop);
            addpoints(h,counter_loop,data(counter_loop));
            
            if (mod(counter_loop , 25) == 0)
                %line(1:counter_loop,data(1:counter_loop));
                drawnow limitrate
                % xlim([0 counter_loop+26]);
                
                %drawnow;
                %  refreshdata
            end
            counter_loop =counter_loop+1;
            %drawnow;
        end
    end
    disp("fim do codigo");
    fclose(t);
end

%%
close all
clear all

mypi = raspi('192.168.0.132','pi','raspberry');


%%
close all

%getFile(mypi,'drone_ws/src/drone/src/Data/Matlab*');
figure

h = plot(NaN,NaN);
setappdata(gca,'LegendColorbarManualSpace',1);
setappdata(gca,'LegendColorbarReclaimSpace',1);
ylim([-50 50]);

while (1)

    %system('scp \\192.168.0.132\Anonymous\src\drone\src\Data\Matlab.txt C:\Users\Guitou\Desktop\drone_ws\src\drone\src\Data\')
   % MATLAB =   load( '\\192.168.0.132\Anonymous\src\drone\src\Data\Matlab.txt');
    getFile(mypi,'/home/pi/drone_ws/src/drone/src/Data/Matlab.txt');
   %try 
       MATLAB =   load( 'Matlab.txt');
       t = size(MATLAB);
       set(h,'XData',1:t,'YData',MATLAB);
       %plot(MATLAB);
       drawnow update;
   %end

   pause(1);
end
