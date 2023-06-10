N=zeros(22560,2);
for x=1:188
    for y=1:120
        N((x-1)*120+y,1)=x;
        N((x-1)*120+y,2)=y;
    end
end
undistortedPoints=  undistortFisheyePoints(N,cameraParams.Intrinsics);
M=[N undistortedPoints];
I=[undistortedPoints N];