load("data.csv");
load("test1.csv");


plot(data(:,1),data(:,2),'k-','Linewidth',2)
hold on
plot(test1(:,3),test1(:,4),'r--')