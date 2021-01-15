function [f]= ROS_spm_fx_robot_adem_reach(x,v,a,P)




a = full(a);
x = full(x);



k3=4;   % viscosity
k4 =3.21;
visco = 2;
elasticity=1.5; % elasticity
m1=4;
m2=3;


   
 f  = [x(8);     % velocities
        x(9);
        x(10);
        x(11);
        x(12);
        x(13);
        x(14);
       (a(1)-visco*k3*x(8)-elasticity*(x(1)))/m1;     
       (a(2)-visco*k3*x(9)-elasticity*(x(2)))/m1;
       (a(3)-visco*k3*x(10)-elasticity*(x(3)))/m1;
       (a(4)-visco*k4*x(11)-elasticity*(x(4)))/m2;
       (a(5)-visco*k4*x(12)-elasticity*(x(5)))/m2;  
       (a(6)-visco*k4*x(13)-elasticity*(x(6)))/m2;
       (a(7)-visco*k4*x(14)-elasticity*(x(7)))/m2];
   
   
   
   
   
   
    
