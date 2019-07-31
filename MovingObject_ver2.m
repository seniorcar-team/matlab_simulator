classdef MovingObject_ver2
    %UNTITLED2 このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties
        position             %[m]
        speed                %[m/s]
        direction            %[deg] unclockwise from x axis
        target_position      %[m] 12/10
        velocity             %[m/s]
        tmp_position         %[m]
        estimated_position   %[m]
        walking_cycle        %[]
        visible
    end
    
    methods
        function MO = MovingObject_ver2( position, speed, direction,target_position )
            % position [m]
            % speed[km/h]
            % direction [deg] unclockwise from x axis
            MO.position = position;%[m]
            MO.speed = speed; %[m/s]
            MO.direction = direction;%[deg]
            MO.target_position = target_position;%[m]12/10
            velocity = [ speed*cos(deg2rad(direction)), speed*sin(deg2rad(direction)) ];%[vx, vy] [km/h]
            MO.velocity = velocity;%[m/s]
            MO.tmp_position = position;%[m]
            MO.estimated_position = position;%他の関数で使うために構造として作っておく
            MO.walking_cycle = randi(5);
            MO.visible = 0;
        end
        
        function obj = plusStep( obj, dt )%現在位置の更新
            obj.tmp_position =  obj.tmp_position + obj.velocity.*dt;
        end
        
        function plotPosition(obj,typeString)
            plot( obj.tmp_position(1), obj.tmp_position(2), typeString );
        end
        
        function obj = calcPosition( obj, now_time )
            obj.tmp_position =  obj.position + obj.velocity.*now_time;
        end
    end
    
end

