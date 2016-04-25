% set up movie recording:
nframes = 76;
vidObj = VideoWriter('JohnKellyRBE520StairVideo.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 25;
open(vidObj);


for i = 1:76
 figure(1);
 %h = figure(1);
plot(tr_posx(i),tr_posy(i),'ko',lknx(i),lkny(i),'r.',lftx(i),lfty(i),'r.',rknx(i),rkny(i),'b.',rftx(i),rfty(i),'b.');
hold on
plot(stairx,stairy,'k')
axis([-1 2 0 3])

plot([tr_posx(i),lknx(i)]',[tr_posy(i),lkny(i)]','r')
plot([lknx(i),lftx(i)]',[lkny(i),lfty(i)]','r')
plot([tr_posx(i),rknx(i)]',[tr_posy(i),rkny(i)]','b')
plot([rknx(i),rftx(i)]',[rkny(i),rfty(i)]','b')
hold off
pause(0.04)
writeVideo(vidObj, getframe);
end

close(vidObj);
