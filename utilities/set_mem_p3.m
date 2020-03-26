function [x_next_zonotope]=set_mem_p3(F_bar ,Q,x_zonotope)

new_center = F_bar*x_zonotope.center ;
new_gen = [F_bar*x_zonotope.generators, Q];
x_next_zonotope = zonotope([new_center, new_gen]); % to be tested

end
