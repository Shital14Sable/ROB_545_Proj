num_of_env = 2;
num_of_itr_per_combo = 10;
num_of_planner = 2;
%save('Test_1_individual-execution-time_env-1-2_planner-1_without-display', 'execution_time') 
Empty = []; 
writematrix(Empty,'Results.xlsx'); 
for i= 1:num_of_env 
    for j = 1:num_of_planner 
        S = csvread('Results.xlsx'); 
        V = [S ; Execution_time(i, j, :)]; 
        writematrix(V,'Results.xlsx','Sheet',i) 
    end
end