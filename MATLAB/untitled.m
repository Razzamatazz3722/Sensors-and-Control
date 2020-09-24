I = imread("test_screenshot.jpg");
%imshow(I);

I_gs = convertRGBtoGS(I);
I_bw = convertGStoBW(I_gs, 0.1);

%imshow(I_bw)


imwrite(I_bw , ['../MATLAB/',  'robot', '.jpg'],'jpg');

new_I = imread("robot.jpg");

[msg,~,loc] = readBarcode(new_I,"QR-CODE");
Imsg = insertShape(new_I, "FilledCircle", [loc, repmat(10, length(loc), 1)],"Color","red","Opacity",1);
imshow(Imsg) 