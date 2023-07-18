%-------------------------------------------%
% BEGIN: function UAS_fixed_targetEndpoint.m %
%-------------------------------------------%
function output = UAS_fixed_targetEndpoint(input)

tf               = input.phase(1).finaltime;


output.objective = tf;

% K                = input.phase.integral;
% beta = 0.7;
% output.objective = beta*K+(1-beta)*tf;

%-------------------------------------------%
% END: function UAS_fixed_targetEndpoint.m   %
%-------------------------------------------%

