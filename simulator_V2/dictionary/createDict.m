arucoDictRaw = dlmread("aruco4x4.csv");

arucoDict = false(size(arucoDictRaw,1),16);

for i = 1:size(arucoDictRaw, 1)

    b = dec2bin(arucoDictRaw(i,:),8)';
    arucoDict(i,:) = logical(b(:)'-'0');
    
end

save("arucoDict")