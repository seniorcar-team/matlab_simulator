function [evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian)

evalDB=[];
trajDB=[];
p_ped =zeros(2,length(Pedestrian));

for vt=Vr(1):Kinematic(5):Vr(2) % vt=velocity
    for ot=Vr(3):Kinematic(6):Vr(4) % ot=steer angle
        %�O�Ղ̐���
        [xt,traj]=GenerateTrajectory(x,vt,ot,PARAM(6),Kinematic);
        %�e�]���֐��̌v�Z
        
        %�������ɕ]���֐��������@2018.10.12.16:38
        heading = CalcHeadingEval(xt,goal);%���K���͂��ĂȂ��C���Ƃł܂Ƃ߂Đ��K��
        vel = CalcVelEval(vt);%���K���͂��ĂȂ��C���Ƃł܂Ƃ߂Đ��K��
        for i = 1:length(Pedestrian)
            p_ped(:,i) = [Pedestrian(i).tmp_position(1);Pedestrian(i).tmp_position(2)];
        end
        p_m_dist = p_ped - xt(1:2);
        dist =  sqrt(min(sum(p_m_dist.*p_m_dist)));%�ǂɊւ��Ă͂��������Ԃ���Ȃ��悤�ɂ����̂Ől������OK
        dir_sp = calc_dir_sp(x,Pedestrian,xt);%���̊֐������@2018/10/12.19:17
        vel_sp = calc_vel_sp(x,Pedestrian,xt);%���̊֐������@2018/10/12.19:17
        
        %�������ɕ]���֐�������  2018.10.12.16:38
        
        %����\���x
        possible_value = possible_velocity([vt ot]);
        
        %���S���̔���(0 or 1, 0=danger)     
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