function [evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian)

evalDB=[];
trajDB=[];
p_ped =zeros(2,length(Pedestrian));

for vt=Vr(1):Kinematic(5):Vr(2) % vt=velocity
    for ot=Vr(3):Kinematic(6):Vr(4) % ot=steer angle
        %‹OÕ‚Ì„’è
        [xt,traj]=GenerateTrajectory(x,vt,ot,PARAM(6),Kinematic);
        %Še•]‰¿ŠÖ”‚ÌŒvZ
        
        %«‚±‚±‚É•]‰¿ŠÖ”‚ğ‘‚­@2018.10.12.16:38
        heading = CalcHeadingEval(xt,goal);%³‹K‰»‚Í‚µ‚Ä‚È‚¢C‚ ‚Æ‚Å‚Ü‚Æ‚ß‚Ä³‹K‰»
        vel = CalcVelEval(vt);%³‹K‰»‚Í‚µ‚Ä‚È‚¢C‚ ‚Æ‚Å‚Ü‚Æ‚ß‚Ä³‹K‰»
        for i = 1:length(Pedestrian)
            p_ped(:,i) = [Pedestrian(i).tmp_position(1);Pedestrian(i).tmp_position(2)];
        end
        p_m_dist = p_ped - xt(1:2);
        dist =  sqrt(min(sum(p_m_dist.*p_m_dist)));%•Ç‚ÉŠÖ‚µ‚Ä‚Í‚»‚à‚»‚à‚Ô‚Â‚©‚ç‚È‚¢‚æ‚¤‚É‚µ‚½‚Ì‚Ål‚¾‚¯‚ÅOK
        dir_sp = calc_dir_sp(x,Pedestrian,xt);%‚±‚ÌŠÖ”‚ğì‚é@2018/10/12.19:17
        vel_sp = calc_vel_sp(x,Pedestrian,xt);%‚±‚ÌŠÖ”‚ğì‚é@2018/10/12.19:17
        
        %ª‚±‚±‚É•]‰¿ŠÖ”‚ğ‘‚­  2018.10.12.16:38
        
        %‘€ì‰Â”\‘¬“x
        possible_value = possible_velocity([vt ot]);
        
        %ˆÀ‘S«‚Ì”»’è(0 or 1, 0=danger)     
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