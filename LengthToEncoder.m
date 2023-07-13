function [ax_counts ax_length_relative]=LengthToEncoder(hex_setup,ax_length)
for i=1:size(ax_length,1)
ax_length_relative(i,:)=ax_length(i,:)-hex_setup.Actuators.DatumLength_Individual(i);
ax_counts(i,:)=ax_length_relative(i,:)*hex_setup.Actuators.CountsPerM;
end