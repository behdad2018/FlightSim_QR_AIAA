%     Copyright (C) 12/12/2018  Regents of the University of Michigan
%     Aerospace Engineering Department
%     Computational Aeroscience Lab, written by Behdad Davoudi

%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.

function [T,Q,P]=Propeller(rpm,V_rel_B)
%#codegen
global geometry

R=geometry(1,1);
nb=geometry(2,1);
A=geometry(3,1);
rho=geometry(4,1);
nr=geometry(5,1);
npsi=geometry(6,1);
th=geometry(7,1:nr);
c=geometry(8,1:nr);
cla=geometry(9,1:nr);
r=geometry(10,1:nr);
psi=geometry(11,:);

% Individual engine code a typical quad-copter (only one of the prop). This
% code provides Thrust, Torque and Power generated/required for a given
% flight condition variables and copter geometric characterstics -- written by B.
% Davoudi, 2017 - Computational Aerosciences Laboratory at University of
% Michigan

% Vx,Vy,Vz               % the relative velocity of the copter wrt wind

cm=mean(c,2);            % maean chord  
sig=nb*cm/(pi*R);        % solidity
om=rpm*2*pi/60;          % rad per second
Vt=om*R;                 % tip velocity
 
% note what in general what is comming inside from simulink has a negative sign for
% z that must be fixed aero model, so when V_rel_B(3) is the velocity of the
% vehicle seen by an observer on the wind, it is negative when the vehicle
% is climbing but the (climb ratio) lambc has to be positive!

Vx = V_rel_B(1);
Vy = V_rel_B(2);
Vz = V_rel_B(3);

% note that in climb ratio we need -Vz brcause we need the vlocity of the
% vehicle not the relative wind to the body

lamc=-Vz/Vt ;                       % climb inflow ratio

rmin=1/nr;

%% BEMT to be used to find the bound vortex on the disk

% Prandtl's tip-loss function
r(end)=0.99;r(1)=rmin;
lam=zeros(1,nr);
phi=lam;
F=phi;

for i=1:length(r)
    Fold=1;dF=1;Fnew=1;
    while abs(dF)>.001
        
        % Eqn. 3.131, page 146 of Leishman
        lam(i)=sqrt((sig*cla(i)/(16*Fold)-lamc*0.5)^2 + sig.*cla(i)*th(i)*r(i)*0.125/Fold)- (sig*cla(i)/(16*Fold)-lamc*0.5);
        phi(i)=atan(lam(i)/r(i));
        froot=0.5*nb*r(i)/((1-r(i))*phi(i));
        ftip=0.5*nb*(1-r(i))/(r(i)*phi(i));
        
        %% based on pp 142 and 144 of Leishman -- does not work
        %  Fnew=(2/pi)*acos(exp(-froot*ftip))
        
        %% My propossal  
        Fnew=(2/pi)*acos(exp(-froot))*(2/pi)*acos(exp(-ftip));
        
        %%
        dF=abs(Fnew-Fold);
        Fold=Fnew;         
    end
   
    F(i)=Fnew;
end

cl=cla.*(th-phi);

% thrust calculations

% L=cl.*0.5*1.225.*sqrt((r*R*om).^2+Vz^2+(lam*R*om).^2).^2.*c;
% T=trapz(r*R,L)*nb;

% cutting due to root:

beg=floor(nr*.1);

L=cl(beg:end).*0.5*1.15.*sqrt((r(beg:end)*R*om).^2+(lam(beg:end)*R*om-Vz).^2).^2.*c(beg:end);
T=trapz(r(beg:end)*R,L)*nb;

% plot(r*R,L);

% exterted forced in x,y, and directions
% Tx=T*cos(alpha)
% Ty=T*cos(beta)
% Tz=T*cos(gamma)

%% Torque  -- July 2017

% blade profile drag
% Cd0=0.0081-0.0216*aoa+0.4*aoa^2; from helicopter literature, note aoa is the angle of attack - page 14, chapter 2 of Friedmann notes
Cd0=0.008;

% thrust coefficient
Ct=T/(rho * pi * R^2 * Vt^2);

% f is the equivalent flat plate area that copter frame occupies in space
% -- note that a quarter of the area should be used since this is model for
% for a helicopter, f/A is between 0.004 to 0.025

f=0.005*A * 0.25;

% mu is the advance ratio

mu= sqrt(Vx^2+Vy^2) / Vt ;

% total power required based on page 80, chapter 2 of Friedmann notes
% page 227 of Leishman

% sig * Cd0 * 0.125 * (1 + 4* mu^2 + 5/8*mu^4)

Cp = sig * Cd0 * 0.125 * (1 + 4.6* mu^2) + ...
    1.15 * 0.5 * Ct^2 / sqrt(mu^2+mean(lam)^2) + ...
    f * mu^3 /(2*pi*R^2) ...
    + Ct*lamc ;

Cq = Cp;

Q = Cq * rho * pi * R^2 * Vt^3 / om;

P=Q*om;


end