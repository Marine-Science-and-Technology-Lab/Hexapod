clear all; close all
[fname fpath]=uigetfile('.mat')

dt=1/256;
load([fpath fname])
useries=MultiSine_Excitation.u;
Rfields=fieldnames(useries);

for nr=1:length(Rfields)

    Rfield=Rfields{nr};
Efields=fieldnames(useries.(Rfield));

for ne=1:length(Efields)
    Efield=Efields{ne};

    pose_t_temp=useries.(Rfield).(Efield)
    N_t=size(pose_t_temp,2)
    t_temp=[1:N_t]'*dt;


    hex_path.pose_t=pose_t_temp;
    hex_path.T=t_temp;

    hex_path.dt=dt;

fnameout=[fname(1:end-4) '_n' num2str((nr-1)*length(Efields)+ne) '_' Rfield '_' Efield '.mat']

mkdir([fpath '\' fname(1:end-4) '_converted'])
    save([fpath '\' fname(1:end-4) '_converted\' fnameout])
end
end


    