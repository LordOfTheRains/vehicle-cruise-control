clear;
clc;


%=====================CONSTANTS===================
passengers = 5;
[vehicleConstants] = VehicleConstants(passengers);
%=====end of constants



%=======Read from xlsx======
filename = 'Module 2 Validation.xlsx';
validationSets = xlsread(filename, 2, '', 'basic');

trimming = 1;


if (trimming)
    for i=1:18%final column is 18
        validationData=validationSets(:,i);
        trimSpeed = validationData(1);%given
        disturbance = validationData(2:4);%given
        vehicleConstants = validationData(25:41);%given
        %{
        controls = validationData(5:6);%expected output
        states = validationData(7:11);%expected output
        output = validationData(12:19);%expected output
        stateDerivatives = validationData(20:24);%expected output
        %}

        [trimmed_state, trimmed_output, trimmed_control,trimmed_Xdot] = ...
            TrimVehicle(trimSpeed, vehicleConstants, disturbance);
        %[trimmed_state, trimmed_output, trimmed_control,trimmed_Xdot] = ...
        %TrimVehicle(25, vehicleConstants, [0 0 0 ])
        
        trim_solution(5:6,i) = trimmed_control;
        trim_solution(7:11,i) = trimmed_state;
        trim_solution(12:19,i) = trimmed_output;
        trim_solution(20:24,i) = trimmed_Xdot;
        
        Xtrim = trimmed_state;
        Utrim = trimmed_control;
        const = vehicleConstants;
        w = disturbance;
        
        if (i == 1)
            
            options = simset('SrcWorkspace','current');
            sim('trim_linearize',[],options);
            [xdot,y] = VehicleDynamicsModel(Xtrim,Utrim,w,const);
            
        end
    end
    
    trim_error = abs(validationSets(5:24,:) - trim_solution(5:24,:));
    xlRange = 'D6';
    solution = array2table(trim_solution(5:24,:));
    trim_error = array2table(trim_error);
    %write result to worksheet 2
    writetable(solution,filename,'Sheet','Trim result','Range','D6', 'WriteVariableNames',false);
    %xlswrite(filename,solution,2,xlRange);
    %write error to worksheet 3
    
    writetable(trim_error,filename,'Sheet','Trim error', 'Range','D6', 'WriteVariableNames',false);
end
passengers = 5;
vkmh=112;
[vehicleConstants] = VehicleConstants(passengers);
states = [0;31.1111111111111;106.73188666888;108.630485983758;389.784468839506];
controls = [0.313647972436715;0];
disturbance = [0;0;0];
[A, Bu,Bw, C, Du,Dw] = LinearizeVehicle(states, controls, disturbance, vehicleConstants)

