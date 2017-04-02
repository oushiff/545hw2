function thetad = inverse_kinematics(u)
    n = (length(u)-2)/2;
    theta = u(1:n);
    xd    = u(n+1:n+2);
    links = u(n+3:end);
    n = length(theta);
    m = length(xd);
    
    theta_sum = 0;   
    x_array = [];
    y_array = []; 
    x_array_sum = [];
    y_array_sum = [];
    alpha = 1;  
    index = 1;
    while index <= n
        theta_sum = theta_sum + theta(index);
        line = links(index);
        delta_x = -line * sin(theta_sum);
        x_array = [x_array [0]];
        x_array = x_array + delta_x;
        x_array_sum = [x_array_sum [0]];
        x_array_sum = x_array_sum + x_array;
        
        delta_y = line * cos(theta_sum);
        y_array = [y_array [0]];
        y_array = y_array + delta_y;
        y_array_sum = [y_array_sum [0]];
        y_array_sum = y_array_sum + y_array;
        index = index + 1;     
    end     
    J = (1/n) * [x_array_sum; y_array_sum];
    J_t = J'; 
    thetad = alpha * J_t * inv(J * J_t) * [xd(1); xd(2)];
end




