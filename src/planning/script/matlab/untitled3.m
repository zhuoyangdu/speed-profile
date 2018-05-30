close all;
figure(1);
i=imread('../../../simulation/data/car.png');
pp=imshow(i);
x=1:10;
y=x+4;
h=plot(x,y);

figure(2);
 load clown
surface(peaks,flipud(X),'FaceColor','texturemap','EdgeColor','none','CDataMapping','direct')
hold on
plot(1:60,'LineWidth',10,'Color','r')

figure(3)
ha=axes('units','normalized','position',[0 0 1 1]); 
uistack(ha,'down') 
II=imread('../../../simulation/data/car.png'); 
image(II) 
colormap gray 
set(ha,'handlevisibility','off','visible','off'); 

figure(4);
ha=axes('units','normalized','position',[0.5 0 0.1 0.5]);
uistack(ha,'top');
II=imread('../../../simulation/data/car.png');

image(II);
hb=axes('units','normalized','position',[0.1 0 0.1 0.1]);




iii=imread('../../../simulation/data/car.png');
image(iii);
colormap gray;
set(ha,'handlevisibility','off','visible','off');
set(hb,'handlevisibility','off','visible','off');
x=-pi:0.1:pi;
y=x.*sin(x.*cos(x)).*tan(x);
plot(x,y,'LineWidth',2)
set(gca,'color','none')
    
    %plot(LOC(1,num),LOC(2,num),'dy','MarkerFaceColor','r','MarkerSize',13);
    %plot(LOC(1,num),LOC(2,num),'dy','MarkerFaceColor','r','MarkerSize',13);
    axis([0.23, 8.63, 0,6.3]); %axis range
    set(gca,'xtick',[0.23:1.2:8.63]);%x axis step set
    set(gca,'ytick',[0:0.9:6.3]);
    set(gca,'color','none');