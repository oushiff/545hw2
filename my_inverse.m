aaaaa = inverse_kinematics([1 1 1 2 2 1 1 1]);
aaaaa;

function thetad = inverse_kinematics(u)
    % performs inverse kinematics computations with various methods

    % Stefan Schaal, March 2009

    % split the input u into meaningful quantities

    n = (length(u)-2)/2;

    theta = u(1:n);
    xd    = u(n+1:n+2);
    links = u(n+3:end);

    n = length(theta);
    m = length(xd);

    theta_sum = 0;
    
    former_jt = [];
    matrix_sum = [];
    
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
        x_array = [x_array [0; 0; 0]];
        x_array = x_array + delta_x;
        x_array_sum = [x_array_sum [0; 0; 0]];
        x_array_sum = x_array_sum + x_array;
        
        delta_y = line * cos(theta_sum);
        y_array = [y_array [0; 0; 0]];
        y_array = y_array + delta_y;
        y_array_sum = [y_array_sum [0; 0; 0]];
        y_array_sum = y_array_sum + y_array;
        index = index + 1;
    end 
    
    J = (1/n) * [x_array_sum; y_array_sum; zeros(n)];

    % convert cartesian velocities into joint velocities

    % NOTE: insert the required inverse kinematics methods at this
    %       location
   
    
    % thetad = alpha * transpose(J) * [xd(1); xd(2); 0];
    thetad = 1;
end




function former_jt = j_transpose(former_jt, line, theta_sum)
    former_jt = vertcat(former_jt, [0 0 0]);
    delta_1 = -line * sin(theta_sum);
    delta_2 = line * cos(theta_sum);
    index = 1;
    len = size(former_jt);
    while index <= len
        former_jt(index, 1) = former_jt(index, 1) + delta_1;
        former_jt(index, 2) = former_jt(index, 2) + delta_2;
        index = index + 1;
    end
end



