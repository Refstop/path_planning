function da = drawArrow(x,y,color)
quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 , 'LineWidth', 2, 'Color', color);
end