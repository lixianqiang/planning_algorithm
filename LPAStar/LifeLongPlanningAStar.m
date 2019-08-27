function LifeLongPlanningAStar()
clear global map Vstart Vend
[openList,closeList]= Init();
while true
    [openList,closeList]=ComputeShortestPath(openList,closeList);
%     [openList,closeList]=BlockVertex(Vcurr,openList,closeList);
end
end

function [openList,closeList]=BlockVertex(Vcurr,openList,closeList)
%���������Ľڵ㣬�����closList���ӵ�openList��rhs=inf,gά�ֲ��䡣����openList�������Ƴ�����ֱ�ӳ�ʼ���ɿհ׵ģ�rhs=inf��g=inf
%ע�⻹Ҫ����k=[k1,k2]��
Vcurr = CalKeyVal(Vcurr);
[openList,closeList]=Operate(closeList,"REMOVE",Vcurr,openList);
end

function [opList,closList]= Init()
startCoord = [1,1];
endCoord = [999,999];

global map Vstart Vend;
map = GenMap([1000,1000],[3,3],600,1,'rand');
Vend   = struct('coord',endCoord,'rhs',inf,'g',inf,'k',[inf,inf],'parent',[]);
Vstart = struct('coord',startCoord,'rhs',0,'g',inf,'k',[inf,inf],'parent',[]);
Vstart = CalKeyVal(Vstart);
opList = [];
opList = [opList Vstart];
closList = struct('coord',[],'rhs',inf,'g',inf,'k',[inf,inf],'parent',[]);
closList(1) =[];
end

function Vcurr = CalKeyVal(Vcurr)
global Vend;
%����ʽ��ֵh�����ڲ���Ҫ���ǵ�ͼ�������������ֱ���þ��뺯������õ�
Vcurr.k(1)=min(Vcurr.g,Vcurr.rhs)+pdist([Vcurr.coord;Vend.coord],'cityblock');
Vcurr.k(2)=min(Vcurr.g,Vcurr.rhs);
end

%�ɱ��������������ڽڵ�Ĵ���ֵ����Ҫ���ǵ�ͼ��������µĴ��ۣ�������Ϊinf����������Ϊ���ڵ��ľ���
%�������㷽������ʹ��Matlab���ڽ�����pdist()����
function costVal = Cost(Vcurr,Vgoal,distMetric)
global map
if nargin < 3
    distMetric = 'manhattan';
end
switch distMetric
    case {'cityblock','manhattan'}
        costVal = abs(Vgoal.coord(1)-Vcurr.coord(1)) + abs(Vgoal.coord(2)-Vcurr.coord(2));
    case 'chebychev'
        costVal = max(abs(Vgoal.coord(1)-Vcurr.coord(1)),abs(Vgoal.coord(2)-Vcurr.coord(2)));
    case 'euclidean'
        costVal = hypot(Vgoal.coord(1)-Vcurr.coord(1),Vgoal.coord(2)-Vcurr.coord(2)); 
    otherwise
       error('Metric distance must be Manhattan, Chebychev, or carbon Euclidean')
%         errInfo = MException('MyComponent:noSuchVariable','Metric distance must be Manhattan, Chebyshev, or carbon Euclidean');
%         throw(errInfo);
end
if map.map(Vgoal.coord(1),Vgoal.coord(2))==1
    costVal=costVal+inf;
end
end

function [opList,closList]=ComputeShortestPath(opList,closList)
global  Vstart Vend;
while(Compare(TopKey(opList),Vend) || Vend.rhs>Vend.g)
    Vcurr=TopKey(opList);
    if Vcurr.g > Vcurr.rhs
        Vcurr.g = Vcurr.rhs;
        [opList,closList] = Operate(opList,"REMOVE_VERTEX",Vcurr,closList);     
        %��ֹ����������������������ж������ŵ���65����70֮��
        %�����дҲû���⣬�ú������һ��ѭ����VendΪTopKey�������ܱ�Vsucc�ڵ�ִ��һ����չ���ȵ���һ�ֵ�TopKey����TopKey>Vend,ѭ����ֹ
        if Vcurr.coord == Vend.coord
            Vend = Vcurr;
            return
        end
        Vsucc = GetNearVertex(Vcurr,opList,closList);        
        for i = 1:length(Vsucc)
            if Vsucc(i).rhs > Vcurr.g+Cost(Vcurr,Vsucc(i),'chebychev')
                Vsucc(i).parent = [Vcurr.coord;Vsucc(i).parent];
                Vsucc(i).rhs = Vcurr.g+Cost(Vcurr,Vsucc(i),'chebychev');
                [opList,closList]=UpdateVetrexs(Vsucc(i),opList,closList);
            end
        end
    elseif Vcurr.rhs > Vcurr.g
        Vcurr.g = inf;
        Vsucc = GetVpredVertex(Vcurr,opList,closList);
        Vsucc = [Vsucc Vcurr]; %����α��������һ�У�����ʵ��������Vcurr��û�н��в��������Ը������ע�͵�
        for i = 1:length(Vsucc)
            if Vsucc(i).coord~=Vstart.coord && isParentVertex(Vsucc(i),Vcurr)   
                [Vsucc(i),opList,closList] = upDateParentVetrex(Vsucc(i),closList,Vcurr);
            end
        end
    end
end
end

function Vout = TopKey(opList)
%��ȫ������openListΪ�յ�ʱ����˳�ComputeShortestPath()����
%һ������£�ֻ��Ŀ��ڵ���ȫ�������ܵ��ﵼ�µ�
if isempty(opList)
    Vout=[];
    return
end

k1=vertcat(opList.k);
topIdx = find(k1(:,1)==min(k1(:,1)));
if length(topIdx)~=1
    k2=vertcat(opList(topIdx).k);
    subIdx = find(k2(:,2)==min(k2(:,2)));
    if length(subIdx)~=1
        subIdx = subIdx(1);
    end
    topIdx = topIdx(subIdx);
end
Vout=opList(topIdx);  
end

function result = Compare(Vcurr,Vgoal)
%��ȫ������openListΪ�յ�ʱ����˳�ComputeShortestPath()����
%һ������£�ֻ��Ŀ��ڵ���ȫ�������ܵ��ﵼ�µ�
if isempty(Vcurr)
    result = false;
    return
end

if Vcurr.k(1)<Vgoal.k(1) || (Vcurr.k(1)==Vgoal.k(1) && Vcurr.k(2)<Vgoal.k(2))
    result = true;
else
    result = false;
end
end

%��ȡ��TopKeyΪ���ĵģ��ܱ߰˸����ڽڵ������չ
%��Ч���ܱ߽ڵ㣺û���г�ʼ���Ŀհ׽ڵ㡢�Ѿ������openList�Ľڵ�
%��Ч���ܱ߽ڵ㣺�����ڵ�ͼ�ϰ����ڵĽڵ㡢�Ѿ������closeList�Ľڵ�
%�������������Ч����Ч��ȫ�������������ıȽϲ�����Ȼ���԰����ǣ���Ч�ڵ㣩ȥ����ֻ���漰��Ч������
function Vsucc = GetNearVertex(Vcurr,opList,closList)
global map
xCen = Vcurr.coord(1);
yCen = Vcurr.coord(2);

xCols = [(xCen>1)*(xCen-1),xCen,(xCen<map.size(1))*(xCen+1)];
xCols = xCols(xCols~=0);

yRows = [(yCen>1)*(yCen-1),yCen,(yCen<map.size(2))*(yCen+1)];
yRows = yRows(yRows~=0);

[X,Y] = meshgrid(xCols,yRows);
xyCoord = [X(:),Y(:)];

%�ڵ�ͼ�ϰ����ڵĽڵ��޳��������ע�͵�
i = map.map(xCols,yRows)==1; 
i=i';
xyCoord(i,:)=[];

[~,idx]=ismember([xCen,yCen],xyCoord,'rows');
xyCoord(idx,:)=[];

openListCoord=vertcat(opList.coord);
closeListCoord=vertcat(closList.coord);
Vsucc=[];
for i=1:size(xyCoord,1)
    if ~isempty(openListCoord)
        [res1,id1]=ismember(xyCoord(i,:),openListCoord,'rows');
    else
        res1=false;
    end
    
    if res1==false && ~isempty(closeListCoord)
        [res2,id2]=ismember(xyCoord(i,:),closeListCoord,'rows');
    else
        res2=false;
    end
    
    if res1==true
        Vsucc = [Vsucc opList(id1)];
    elseif res2==false
        Vsucc = [Vsucc struct('coord',xyCoord(i,:),'rhs',inf,'g',inf,'k',[inf,inf],'parent',[])];
%     else  
%         Vsucc = [Vsucc closList(id2)]; %�Ѿ�������closList�е��ܱ߽ڵ㣬���������ע�͵�
    end
    %��־λ����
    res1=[];
    res2=[];
end
end

function Vsucc = GetVpredVertex(Vcurr,opList,closList,map)
xCen = Vcurr.coord(1);
yCen = Vcurr.coord(2);

xCols = [(xCen>1)*(xCen-1),xCen,(xCen<map.size(1))*(xCen+1)];
xCols = xCols(xCols~=0);

yRows = [(yCen>1)*(yCen-1),yCen,(yCen<map.size(2))*(yCen+1)];
yRows = yRows(yRows~=0);

[X,Y] = meshgrid(xCols,yRows);
xyCoord = [X(:),Y(:)];

[~,idx]=ismember([xCen,yCen],xyCoord,'rows');
xyCoord(idx,:)=[];
openListCoord=vertcat(opList.coord);
closeListCoord=vertcat(closList.coord);
Vsucc=[];
for i=1:size(xyCoord,1)
    if ~isempty(openListCoord)
        [res1,id1]=ismember(xyCoord(i,:),openListCoord,'rows');
    else
        res1=false;
    end
    
    if res1==false && ~isempty(closeListCoord)
        [res2,id2]=ismember(xyCoord(i,:),closeListCoord,'rows');
    else
        res2=false;
    end
    
    if res1==true
        Vsucc = [Vsucc opList(id1)];
    elseif res2==false
        Vsucc = [Vsucc struct('coord',xyCoord(i,:),'rhs',inf,'g',inf,'k',[inf,inf],'parent',[])];
    else  
        Vsucc = [Vsucc closList(id2)]; 
    end
    res1=[];
    res2=[];
end
end

function [Vcurr,opList,closList] = upDateParentVetrex(Vcurr,Vold_parent,opList,closList)
Vcurr.parent=Vold_parent;
Vcurr.rhs=Vold_parent.rhs;
closListCoords=vertcat(closList.coord);
if ~isempty(closListCoords)
    return
end
for i=1:size(Vcurr.parent,1)
        [res,idx]=ismember(Vcurr.parent(i,:),closListCoords,'rows');
        if res==true && Vcurr.rhs > closList(idx).g+Cost(closList(idx),Vcurr,'chebychev')
            Vnew_parent = closList(idx);
            Vcurr.parent = Vnew_parent;
            Vcurr.rhs=Vnew_parent.g+Cost(Vnew_parent,Vcurr,'chebychev');
        end
end
[opList,closList]= UpdateVetrexs(Vcurr,opList,closList);
end

function [opList,closList]= UpdateVetrexs(Vcurr,opList,closList)
if isListMember(Vcurr,opList)
    opList = Operate(opList,'UPDATE_KEY',Vcurr,[]);
elseif ~isListMember(Vcurr,opList) && ~isListMember(Vcurr,closList) % && Vcurr.rhs~=Vcurr.g //�����Vcurr.rhs��Vcurr.g���������ע�ӵ�
    opList = [opList CalKeyVal(Vcurr)];
elseif Vcurr.rhs~=Vcurr.g && isListMember(Vcurr,closList)
    [opList,closList] = Operate(closList,'REMOVE_VERTEX',Vcurr,opList);
end
end

function [popVertList,pushVertList]= Operate(popVertList,opStr,Vcurr,pushVertList)
popListCoord=vertcat(popVertList.coord);
[~,idx]=ismember(Vcurr.coord,popListCoord,'rows'); 
switch upper(opStr)
    case 'REMOVE_VERTEX'
        popVertList(idx) = [];
        pushVertList = [pushVertList Vcurr];
    case 'UPDATE_KEY'
        popVertList(idx)=CalKeyVal(Vcurr);         
end
end

function result = isListMember(Vcurr,opList)
openListCoord=vertcat(opList.coord);
if ~isempty(openListCoord)
    result = ismember(Vcurr.coord,openListCoord,'rows');
else
    result = false;
end
end

function result = isParentVertex(Vcurr,Vparent)
parentCoords=vertcat(Vcurr.parent);
result = ismember(Vparent.coord,parentCoords,'rows');
end