function [imgGS] = convertRGBtoGrayscale_student(imgRGB)

% Get the size of the input image
[rows, cols, channels] = size(imgRGB)

% Create an empty matrix for the new greyscale image
imgGS = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        % Your logic goes in here
         imgGS(i,j) = (imgRGB(i,j,1) + imgRGB(i,j,2) + imgRGB(i,j,3))/3;
    end
end

imgGS = uint8(imgGS);
imshow(imgGS);

end