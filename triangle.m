function tri = triangle(x, y, th)
r=0.3;
x_tri(1) = r*cos(0);
y_tri(1) = r*sin(0);
x_tri(2) = r*cos(2*pi/3);
y_tri(2) = r*sin(2*pi/3);
x_tri(3) = r*cos(-2*pi/3);
y_tri(3) = r*sin(-2*pi/3);

tri = [x_tri;y_tri;ones(1,3)];

T = [cos(th), -sin(th), x;
    sin(th), cos(th), y;
    0, 0, 1];

tri = T*tri;
end