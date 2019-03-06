data=csvread('campus.csv',1,0);

[~,idx] = sort(data(:,8)); 
data_sorted = data(idx,:);
index = find(logical(diff(data_sorted(:,8))));
index(end+1)=size(data,1);
index0=1;
z= find(logical(diff(data(:,1))));
index1 = [1;z;size(data,1)];
for k=1:7954
   a=data_sorted(index0:index(k),:);
   comb = combnk(a(:,1),2);
   for i=1:size(comb)
       p1 = data_sorted(index0 - 1 + find(a(:,1)== comb(i,1)),:);
       p2 = data_sorted(index0 - 1 + find(a(:,1)== comb(i,2)),:);
       p3 = data(index1(comb(i,1)+1)+randi(index1(comb(i,1)+8)-index1(comb(i,1)+7),1)-1,:);
       tau1(k,i)=ttc(p1,p2);
       tau2(k,i)=ttc(p3,p2);
   end
  index0=index(k);
end
[~,~,v1]=find(tau1);
subplot(1,2,1)
histogram(v1,50);
subplot(1,2,2)
histogram(v2,50);


