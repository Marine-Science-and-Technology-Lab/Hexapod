function [iscoll mindist witnesspoints]=checkYokeCollision(Group1,Group2)

N1=length(Group1);
N2=length(Group2);

[ComboMatrix1 ComboMatrix2]=meshgrid(1:N1,1:N2);

ComboArray=[ComboMatrix1(:) ComboMatrix2(:)];

for n=1:length(ComboArray)
    [Coll(n) mindist(n) witness{n}]=checkCollision(Group1(ComboArray(n,1)),Group2(ComboArray(n,2)));
end

iscoll=max(Coll)
if ~iscoll
    [mindist closestpoints]=min(mindist);
    witnesspoints=witness{closestpoints};
else
    mindist=0;
    witnesspoints=[nan nan;nan nan; nan nan];
end
    