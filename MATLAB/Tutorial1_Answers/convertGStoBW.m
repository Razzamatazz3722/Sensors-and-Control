function [imgBW] = convertGStoBW(imgGS, threshold)

% Get the size of the input image
[rows, cols, channels] = size(imgGS);

%create an empty matrix for the binary image
imgBW = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        if imgGS(i,j) >= threshold*256
            imgBW(i,j) = 1;
        end
    end
end

imgBW = logical(imgBW);
%imshow(imgBW);

end