function [x,history] = runopt(x0,ctrl,lb,ub,p,history)
    %% setup and solve nonlinear programming problem
    xLast = [];                     % Last place computeall was called
    myf = [];                       % Used for objective at xLast
    mycineq = [];                   % Used for nonlinear inequality constraint
    myceq = [];                     % Used for nonlinear equality constraint
    problem.objective = @objfun;    %nested below  
    problem.nonlcon = @constr;      %nested below  
    problem.x0 = x0;                        % initial guess for decision variables
    problem.lb = lb;                        % lower bound on decision variables
    problem.ub = ub;                        % upper bound on decision variables
    problem.Aineq = []; problem.bineq = []; % no linear inequality constraints
    problem.Aeq = []; problem.beq = [];     % no linear equality constraints
    problem.options = optimset(...
        'Display','iter-detailed',...
        'Algorithm','sqp',...
        'OutputFcn',@outfun,...
        'TolX',1e-3,...
        'TolCon',1e-3 ...
        );
    problem.solver = 'fmincon';                                 % required for some reason...
    x = fmincon(problem);                 
        function stop = outfun(x,optimValues,state)
         stop = false;
         switch state
             case 'init'
             case 'iter'
             % Concatenate current point and objective function
             % value with history. x must be a row vector.
               history.fval = [history.fval; optimValues.fval];
               history.x = [history.x; x];
             case 'done'
             otherwise
         end
        end
        function y = objfun(x)
            if ~isequal(x,xLast) %check if change
                [myf,mycineq,myceq] = computeall(x,ctrl,p);
                xLast = x;
            end
            %now compute objective
            y = myf;
        end
        function [cineq,ceq] = constr(x)
            if ~isequal(x,xLast) %check if change
                [myf,mycineq,myceq] = computeall(x,ctrl,p);
                xLast = x;
            end
            cineq = mycineq;
            ceq = myceq;
        end
end   
