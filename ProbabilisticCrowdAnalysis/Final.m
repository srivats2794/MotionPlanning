data=csvread('campus.csv',1,0);

[~,idx] = sort(data(:,8)); % sort just the first column
data_sorted = data(idx,:);   % sort the whole matrix using the sort indices
index = find(logical(diff(data_sorted(:,8))));
index(end+1)=size(data,1);
tau1=zeros(size(index,1),1326);
tau2=zeros(size(index,1),1326);
index0=1;
index1 = [1;find(logical(diff(data(:,1))));size(data,1)];

for k=1:size(index,1)
   a=data_sorted(index0:index(k),:);
   comb = combnk(a(:,1),2);
   for i=1:size(comb)
       p1 = data_sorted(index0 - 1 + find(a(:,1)== comb(i,1)),:);
       p2 = data_sorted(index0 - 1 + find(a(:,1)== comb(i,2)),:);
       p3 = [231,231,-1.46200000000000,2.21980000000000,-0.0275016000000000,0.0100017000000000,0.100000000000000,68.5610000000000];
       tau1(k,i)=ttc(p1,p2);
       tau2(k,i)=ttc(p3,p2);
   end
   index0=index(k);
   
end
[~,~,v1]=find(tau1);
[~,~,v2]=find(tau2);
%subplot(1,2,1)
%histogram(v1,50);
%subplot(1,2,2)
%histogram(v2,50);

%% 
numIntervals = 10;
intervalWidth = 0.5; %(max(v1) - min(v1))/numIntervals;
x1 = 0:intervalWidth:8;
ncount = histc(v1,x1);
relativefreq = ncount/length(v1);
plot(x1, relativefreq)
xlim([min(x1) max(x1)])
set(gca, 'xtick', x1)
%% 
numIntervals2 = 10;
intervalWidth2 = 0.5;%(max(v2) - min(v2))/numIntervals2;
x2 = 0:intervalWidth2:8;
ncount2 = histc(v2,x2);
relativefreq2 = ncount2/length(v2);
plot(x2, relativefreq2)
xlim([min(x2) max(x2)])
set(gca, 'xtick', x2)

%%
gtau= relativefreq./relativefreq2;
x3= 0:0.5:8;
plot(x3,gtau)
xlim([min(x3) max(x3)])
set(gca, 'xtick', x3)

%%
u= log(1./gtau);
plot(x3,u)
xlim([min(x3) max(x3)])
set(gca, 'xtick', x3)

%%
filename1= 'IPtau.csv';
filename2= 'NIPtau.csv';
csvwrite (filename1,v1);
csvwrite (filename2,v2);
