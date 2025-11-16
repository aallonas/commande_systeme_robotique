% Modèle géomérique direct du mécanisme étudié dans le cadre du cours d'informatique industielle
% Allonas Alexandre - MIQ5
% 16.10.2025
clear all;
close all;

%% Configuration initiale
[P] = param() ; % sous-prog pour initialiser les constantes

%% Vecteur des coordonnées du point E
Xlin0 = [P.l0a*cos(deg2rad(P.alpha0a))+P.l4*cos(deg2rad(q4))+P.l5a*cos(deg2rad(q5))+P.l5b*cos(deg2rad(q5)) P.l0a*sin(deg2rad(P.alpha0a))+P.l4*sin(deg2rad(q4))+P.l5a*sin(deg2rad(q5))+P.l5b*sin(deg2rad(q5))] ;
X0 = [Xang0 Xlin0] ;            %% Vecteur de tous les paramètres
X0 = X0' ;                      %% Transposée pour vecteur colonne

t = 0 ;                         %% Initialisation du temps à 0
epsAng = [1;1;1;1;1] ;          %% Tolérance sur les angles
epsLin = [1;1] ;                %% Tolérance sur les positions
Epsilon = [epsAng;epsLin] ;     %% Vecteur des tolérances
X = X0 ;                        %% Initialisation du vecteur des paramètres

dt=0.05 ;                       %% Incrément temporel Delta t [s]
tmax=2 ;                       %% Temps de fin de simulation [s]
j = 0 ;                         %% Initialisation de l'index de la boucle de résolution
Xsol = [] ;                     %% Initialisation de la matrice contenant l'ensemble des résultats de géométrie
dotXsol = [] ;                  %% Matrice contenant l'ensemble des résultats de cinématique

while t<tmax %% boucle de résolution
    [X1] = NewtonRaphson(t,X,Epsilon) ;     %% Méthode de résolution
    Xsol = [Xsol X1] ;                      %% On récupère la solution dans Xsol                  
    X = X1 ;                                %% On change de temps
    t = t+dt ;                              %% Incrément du temps
    j = j+1 ;                               %% Incrément de l'itération
end




%% Visualisation de la solution trouvée
bushingRadius       = 5 ;
bushingCircleRadius = 5 ;
jointSize           = 10 ;
screenInfo          = get(0, 'screensize') ;
screenSize          = screenInfo(3:4) ;

%% Ouverture d'une fenêtre
mechPlot = figure('Name', 'Simulation du mécanisme', 'Position', [0 0 0.6*screenSize(1)/2 0.6*screenSize(2)]); 
clf;
axes('position', [.05 0.15 0.9 0.75], 'box','on', ...
    'xcolor','black','ycolor','black', ...
	'fontname', 'times', 'fontsize', 10);
set(gcf, 'color', 'white')
axis([-500 100 -400 200])
grid on

%% Préparation des objets à tracer

% Chargement des paramètres géométriques
[P] = param() ;

% Extraction des données
X = Xsol;
q1 = X(1,:) ;  %% theta_1
q2 = X(2,:) ;  %% theta_2
q3 = X(3,:) ;  %% theta_3
q4 = X(4,:) ;  %% theta_4
q5 = X(5,:) ;  %% theta_5
q6 = X(6,:) ;  %% x_E
q7 = X(7,:) ;  %% y_E

%% Coordonnées des points à tracer
%% Les points fixes
Ax = 0;
Ay = 0;
Bx = P.l0a*cos(P.alpha0a);
By = P.l0a*sin(P.alpha0a);
Cx = P.l0b*cos(P.alpha0);
Cy = P.l0b*sin(P.alpha0);
%% Tracé des liaisons au bâti
drawBushing(Ax,Ay); 
drawBushing(Bx,By); 
drawBushing(Cx,Cy); 

%% Déclaration des objets barres et articulations
jointD = line('xdata', [], 'ydata', [], 'marker', 'o', ...
              'markersize', jointSize, 'color', 'black');
jointF = line('xdata', [], 'ydata', [], 'marker', 'o', ...
              'markersize', jointSize, 'color', 'black');
jointG = line('xdata', [], 'ydata', [], 'marker', 'o', ...
              'markersize', jointSize, 'color', 'black');
jointH = line('xdata', [], 'ydata', [], 'marker', 'o', ...
              'markersize', jointSize, 'color', 'black');

barAD = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barDG = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barCG = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barBH = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barGF = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barHF = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');
barFE = line('xdata', [], 'ydata' , [], 'linewidth' , 2, 'color', 'k');


%% Les points mobiles
Ax = 0;
Ay = 0;
Bx = 
By = 



for i=1:j
    figure(mechPlot);
    set(jointD, 'xdata', Dx(i), 'ydata', Dy(i));
    set(jointF, 'xdata', Fx(i), 'ydata', Fy(i));
    set(jointG, 'xdata', Gx(i), 'ydata', Gy(i));
    set(jointH, 'xdata', Hx(i), 'ydata', Hy(i));
    
    set(barAD, 'xdata', [Ax Dx(i)], 'ydata', [Ay Dy(i)]);
    set(barDG, 'xdata', [Dx(i) Gx(i)], 'ydata', [Dy(i) Gy(i)]);
    set(barCG, 'xdata', [Cx Gx(i)], 'ydata', [Cy Gy(i)]);
    set(barBH, 'xdata', [Bx Hx(i)], 'ydata', [By Hy(i)]);
    set(barGF, 'xdata', [Gx(i) Fx(i)], 'ydata', [Gy(i) Fy(i)]);
    set(barHF, 'xdata', [Hx(i) Fx(i)], 'ydata', [Hy(i) Fy(i)]);
    set(barFE, 'xdata', [Fx(i) Ex(i)], 'ydata', [Fy(i) Ey(i)]);
    
    drawnow
    pause(0.1) %% Temporisation entre 2 instants successifs
end




hold on;
figure(mechPlot);
plot(Ex,Ey);


%% Tracé de la trajectoire de E 
plot_traj_E = figure('Name', 'Trajectoire du point E', 'Position', [0 0 0.6*screenSize(1)/2 0.6*screenSize(2)]);
plot(Ex,Ey);
hXlabel=xlabel('Angle $x_{E}$ (mm)');
hYlabel=ylabel('Angle $y_{E}$ (mm)');
set(hXlabel,'Interpreter','latex');
set(hYlabel,'Interpreter','latex');
hTitle=title('Trajectoire du point $E$');
set(hTitle,'Interpreter','latex');
grid on;


%% Tracé des vitesses de E selon x et y en fct de theta_1

%% Calcul de la matrice dérivée dotXsol par accroissements finis
for g =1:length(Xsol(:,1))
    for h =1:length(Xsol(1,:))-1
        dotXsol(g,h) = (Xsol(g,h+1)-Xsol(g,h))/dt ;
    end
end


dotXE = dotXsol(6,:) ;
dotYE = dotXsol(7,:) ;
temp = length(Xsol(1,:))-1 ;
theta_1 = Xsol(1,1:temp) ;

plot_vitesse = figure('Name', 'Vitesses projetées de E en fonction de theta 1', 'Position', [0 0 0.6*screenSize(1)/2 0.6*screenSize(2)]);
hold on
plot(theta_1*180/pi,dotXE , 'r')
plot(theta_1*180/pi,dotYE , 'b')
hold off
legend('dotx_E','doty_E')
hXlabel=xlabel('Angle $theta_{1}$ (deg)');
hYlabel=ylabel('Vitesses $dot{x}_{E}$ et $dot{y}_{E}$ (mm/s)');
set(hXlabel,'Interpreter','latex');
set(hYlabel,'Interpreter','latex');
hTitle=title('Vitesses du point $E$ en fonction de $\theta_{1}$');
set(hTitle,'Interpreter','latex');

grid on;




%% Tracé de l'évolution de theta_5
theta_5 = Xsol(5,:) ;
t = 0:dt:tmax-dt ;

plot_theta_5 = figure('Name', 'Theta_{5} en fonction du temps', 'Position', [0 0 0.6*screenSize(1)/2 0.6*screenSize(2)]);
plot(t,theta_5*180/pi)
hXlabel=xlabel('Temps (s)');
hYlabel=ylabel('$\theta_{5}$ (deg)');
set(hXlabel,'Interpreter','latex');
set(hYlabel,'Interpreter','latex');
hTitle=title('$\theta_{5}$ en fonction du temps');
set(hTitle,'Interpreter','latex');

grid on;



%% à compléter %%

%% Tracé de l'évolution de dot{theta_5}
%% à compléter %%

