clc
clear
num_of_env = 6;
num_of_itr_per_combo = 10;
num_of_planner = 2;

for a = 1:num_of_env
    for b = 1:num_of_planner
        for c = 1:num_of_itr_per_combo
            file_name = sprintf('Test 2/Test_2_InitObj env%d planner%d itr%d-RRT-Star-Itr-limit.csv' , a, b, c);
            file_name2 = sprintf('Test 2/Test_2_ObjGoal env%d planner%d itr%d-RRT-Star-Itr-limit.csv' , a, b, c);
            Part1 = csvread(file_name);
            Part2 = csvread(file_name2);
            
            Combine = [Part1; Part2];
            
            Part3= zeros(size(Part2));
            %Part4= zeros(size(Part2));
            for s = 1:size(Part2,1) 
            Part3(s,:) = Part2(s,:) - Part2(1,:);
            end
             
            for s = 1:size(Part2,1)-1
            Part4(s,:) = Part2(s+1,:) - Part2(s,:);
            %Combine = [Part1; Part2];
            end
             
            for d = 1:7
                p = figure(a+b+c);
                plot(Part2(:,d), 'DisplayName', ['Joint ', num2str(d)])
                hold on
            end
            xlabel('Timestep') 
            ylabel('Joint Angle')
            legend('show')
            title('Line Plot Joint Angles of Kinova Arm at Each Timestep')
            exportgraphics(p,['Graphs/Test 2/Type1/Test_2_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) 'Type1.png'],'Resolution',600)
            hold off
            
            for d = 1:7
                q = figure(a+b+c+1);
                plot(Part4(:,d), 'DisplayName', ['Joint ', num2str(d)])
                hold on
            end
            xlabel('Timestep') 
            ylabel('Joint Angle')
            legend('show')
            title('Line Plot Joint Angles of Kinova Arm at Each Timestep')
            exportgraphics(q,['Graphs/Test 2/Type2/Test_2_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) 'Type2.png'],'Resolution',600)
            hold off
            
            
            for d = 1:7
                r = figure(a+b+c+2);
                plot(Part3(:,d),  'DisplayName', ['Joint ', num2str(d)])
                hold on
            end
            xlabel('Timestep') 
            ylabel('Joint Angle')
            legend('show')
            title('Line Plot Joint Angles of Kinova Arm at Each Timestep')
            exportgraphics(r,['Graphs/Test 2/Type3/Test_2_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) 'Type3.png'],'Resolution',600)
            hold off
            
            
            for d = 1:7
                n = figure(a+b+c+3);
                plot(Combine(:,d),  'DisplayName', ['Joint ', num2str(d)])
                hold on
            end
            hold off
            xlabel('Timestep') 
            ylabel('Joint Angle')
            legend('show')
            title('Line Plot Joint Angles of Kinova Arm at Each Timestep')
            exportgraphics(n,['Graphs/Test 2/Type4/Test_2_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) 'Type4.png'],'Resolution',600)
        end
    end
end
    