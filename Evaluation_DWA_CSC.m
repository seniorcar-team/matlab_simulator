function [evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian)

evalDB=[];
trajDB=[];
p_ped =zeros(2,length(Pedestrian));

for vt=Vr(1):Kinematic(5):Vr(2) % vt=velocity
    for ot=Vr(3):Kinematic(6):Vr(4) % ot=steer angle
        %軌跡の推定
        [xt,traj]=GenerateTrajectory(x,vt,ot,PARAM(6),Kinematic);
        %各評価関数の計算
        
        %↓ここに評価関数を書く　2018.10.12.16:38
        heading = CalcHeadingEval(xt,goal);%正規化はしてない，あとでまとめて正規化
        vel = CalcVelEval(vt);%正規化はしてない，あとでまとめて正規化
        for i = 1:length(Pedestrian)
            p_ped(:,i) = [Pedestrian(i).tmp_position(1);Pedestrian(i).tmp_position(2)];
        end
        p_m_dist = p_ped - xt(1:2);
        dist =  sqrt(min(sum(p_m_dist.*p_m_dist)));%壁に関してはそもそもぶつからないようにしたので人だけでOK
        dir_sp = calc_dir_sp(x,Pedestrian,xt);%この関数を作る　2018/10/12.19:17
        vel_sp = calc_vel_sp(x,Pedestrian,xt);%この関数を作る　2018/10/12.19:17
        
        %↑ここに評価関数を書く  2018.10.12.16:38
        
        %操作可能速度
        possible_value = possible_velocity([vt ot]);
        
        %安全性の判定(0 or 1, 0=danger)     
        safe_wall = CalcWallCheck(traj);
        safe_ped_static = Judgement_margin_2_time(traj, Pedestrian);
        %safe_ped_crossing = Judgement_crossing_3_2(x,[vt ot],Pedestrian);
        %safe_ped = min(safe_ped_static,safe_ped_crossing);
        
%         if ~safe_ped
%             disp('danger with ped')
%         end
        %collision_safety = min(safe_wall,safe_ped);
        collision_safety = min(safe_wall,safe_ped_static);
        safety = min(possible_value,collision_safety);
        if safety == 0
            evalDB = [evalDB;[vt ot 0,0,0,0,0]];
        else
            EVAL = [heading,dist,vel,dir_sp,vel_sp];
            evalDB=[evalDB;[vt ot EVAL]];
        end
        trajDB=[trajDB;traj];     
    end
end