function two_link_anim(Os, filename)
 %filename = 'two_link_anim1.gif';
 figure('Renderer', 'painters', 'units', 'pixels', 'Position', [100 100 500 500])
 for i = 1:length(Os(:, 1))
 clf
 axis equal
 xlim([-2, 2]);
 ylim([-2, 2]);
 hold on
 plot(0, 0, '*k')
 hold on
 plot([0; Os(i, 1),; Os(i, 3)], [0; Os(i, 2); Os(i, 4)], 'o-');

 drawnow
 frame = getframe(gcf);
 im = frame2im(frame);
 [imind,cm] = rgb2ind(im,256);
 if i == 1
 imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
 else
 imwrite(imind,cm,filename,'gif','WriteMode','append');
 end
 end
end
