classdef Pedestrian_ver3
    %Pedestrian 等速直線運動歩行者の移動体クラス
    %   初期位置、初期速度,目標位置を指定。
    
    properties
        position             %[m]
        speed                %[m/s]
        direction            %[deg] unclockwise from x axis
        target_position             %[m]
        velocity             %[m/s]
        tmp_position         %[m]
        estimated_position   %[m]
        walking_cycle        %[]
    end
    
    methods
        function PD = Pedestrian(position, speed, direction, target_position)
            PD.position = [max([-25 min([25 position(1)])]) max([-2 min([2 position(2)])])];%[m]
            PD.speed = speed./3.6;%[m/s]
            PD.direction = direction;%[deg]
            PD.target_position = target_position;%目標位置を格納
            tmp_velocity = [ speed*cos(deg2rad(direction)), speed*sin(deg2rad(direction)) ];%[vx, vy] [km/h]
            PD.velocity = tmp_velocity./3.6;%[m/s]
            PD.tmp_position = position;%[m]
            PD.estimated_position = position;%[m]
            PD.walking_cycle = randi(5);
        end
        
        %function obj = plusStep( obj, dt )
         %   obj.tmp_position =  obj.tmp_position + obj.velocity.*dt;
        %end
        
        %function plotPosition(obj,typeString)
        %    plot( obj.tmp_position(1), obj.tmp_position(2), typeString );
        %end
        
        %function obj = calcPosition( obj, now_time )
        %    obj.tmp_position =  obj.position + obj.velocity.*now_time;
        %end
    end
    
end

