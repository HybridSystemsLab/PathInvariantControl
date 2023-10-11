function [bool] = unsafeSetCircle(x,y)
%UNSAFESET Returns true if value is in unsafe set
% U = [1,0,0.2,0.2;
%      -0.5,0.5,0.2,0.2;
%      0.2,0.6,0.2,0.2;
%       2.5, 0, 0.5, 0.5];

bool = false;

if( (x>=1 && x<=1.2) && (y>=0 && y<=0.2) )
    bool = true;
end
if( (x>=-0.5 && x<-0.3) && (y>=0.5 && y<=0.7) )
    bool = true;
end
if( (x>=0.2 && x<=0.4) && (y>=.6 && y<=0.8) )
    bool = true;
end
if( (x>=2.5 && x<=3) && (y>=0 && y<=0.5) )
    bool = true;
end

if ( x>3.5 || x<-3.5 || y>2 || y<-2 )
    bool = true;
end


end