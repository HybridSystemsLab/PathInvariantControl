function [bool] = unsafeSet(x,y)
%UNSAFESET Returns true if value is in unsafe set
% U = [1,2,0.5,0.5;
%      3,2.5,0.5,0.5;
%      4,1,0.5,0.5];

bool = false;

if( (x>=1 && x<=1.5) && (y>=2 && y<=2.5) )
    bool = true;
end
if( (x>=3 && x<=3.5) && (y>=2.5 && y<=3) )
    bool = true;
end
if( (x>=4 && x<=4.5) && (y>=1 && y<=1.5) )
    bool = true;
end


end

