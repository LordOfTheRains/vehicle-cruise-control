function [A, Bu,Bw, C, Du,Dw] = LinearizeVehicle(X_bar, u_bar, w_bar, constants)


delta= 0.1;
for index=1:5
%perturb Xbar


    %reset to solution before perturbation
    xperturbed = X_bar;
    
    
    dh = xperturbed(index)*(1+delta);
    if (dh == 0)
        dh = delta;
    end
    %/ perturb high
    xperturbed(index) =  xperturbed(index)+dh;
    perturb_delta = delta;
    %run the model
    [x_high, y_high] = VehicleDynamicsModel(xperturbed, u_bar, w_bar, constants);

    
    %reset to solution before perturbation
    xperturbed = X_bar;
    %/ perturb low
    xperturbed(index) = xperturbed(index)-dh;
    %run the model
    [x_low, y_low] = VehicleDynamicsModel(xperturbed, u_bar, w_bar, constants);
    A(:,index) = 0.5/perturb_delta*(x_high-x_low);
    C(:,index) = 0.5/perturb_delta*(y_high-y_low);
end

for index=1:2
%perturb ubar


    %reset to solution before perturbation
    uperturbed = u_bar;
    
    
    dh = uperturbed(index)*(1+delta);
    if (dh == 0)
        dh = delta;
    end
    
    %/ perturb high
    uperturbed(index) =  uperturbed(index)+dh;
    perturb_delta = uperturbed(index)*delta;
    %run the model
    [x_high, y_high] = VehicleDynamicsModel(X_bar, uperturbed, w_bar, constants);

    
    %reset to solution before perturbation
    uperturbed = u_bar;
    %/ perturb low
    uperturbed(index) = uperturbed(index)-dh;
    %run the model
    [x_low, y_low] = VehicleDynamicsModel(X_bar, uperturbed, w_bar, constants);
    Bu(:,index) = 0.5/perturb_delta*(x_high-x_low);
    Du(:,index) = 0.5/perturb_delta*(y_high-y_low);
end


for index=1:3
%perturb wbar
    %reset to solution before perturbation
    wperturbed = w_bar;
    

    dh = wperturbed(index)*(1+delta);
    if (dh == 0)
        dh = delta;
    end
    %/ perturb high
    wperturbed(index) =  wperturbed(index)+dh;
    perturb_delta = wperturbed(index)*delta;
    %run the model
    [x_high, y_high] = VehicleDynamicsModel(X_bar, u_bar, wperturbed, constants);

    
    %reset to solution before perturbation
    wperturbed = w_bar;
    %/ perturb low
    wperturbed(index) = wperturbed(index)-dh;
    %run the model
    [x_low, y_low] = VehicleDynamicsModel(X_bar, u_bar, wperturbed, constants);
    Bw(:,index) = 0.5/perturb_delta*(x_high-x_low);
    Dw(:,index) = 0.5/perturb_delta*(y_high-y_low);
end



end