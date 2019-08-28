function LifeLongPlanningAStar()
clear global map Vstart Vend
[openList,closeList]= Init();
while true
    [openList,closeList]=ComputeShortestPath(openList,closeList);
    [openList,closeList]=BlockVertex(openList,closeList);
end
end

function [openList,closeList]=BlockVertex(openList,closeList)
%这里只是简单验证当节点阻塞时，LPA*是如何进行再扩展
%将被阻塞的节点，如果在closList就扔到openList，rhs=inf,g维持不变。而在openList从里面移出来，直接初始化成空白的，rhs=inf，g=inf
%注意还要更新k=[k1,k2]的
ux=11; uy=14;
dx=15; dy=25;
map.map(ux:dx,uy:dy)=1;
for i=ux:dx
    for j=uy:dy
        Vcurr.coord=[i,j];
        [result,idx] = ismember(Vcurr.coord,vertcat(closeList.coord),'rows');
        if result==true
            Vcurr=closeList(idx);
            Vcurr.rhs=inf;
            [closeList,openList]=Operate(closeList,"REMOVE_VERTEX",Vcurr,openList);
        end
    end
end
end
    
function [opList,closList]= Init()
startCoord = [2,5];
endCoord = [99,99];
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
%启发式数值h，由于不需要考虑地图阻塞情况，可以直接用距离函数计算得到
Vcurr.k(1)=min(Vcurr.g,Vcurr.rhs)+pdist([Vcurr.coord;Vend.coord],'chebychev');
Vcurr.k(2)=min(Vcurr.g,Vcurr.rhs);
end

%成本函数是用于相邻节点的代价值，需要考虑地图阻塞情况下的代价，若阻塞为inf，若不阻塞为两节点间的距离
%其距离计算方法可以使用Matlab的内建函数pdist()代替
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
        %终止条件触发，根据情况将该判断条件放到行65到行70之间
        %如果不写也没问题，该函数最后一次循环以Vend为TopKey，对其周边Vsucc节点执行一次扩展，等到下一轮的TopKey满足TopKey>Vend,循环终止
        if Vcurr.coord == Vend.coord
            Vend = Vcurr;
            return
        end
        Vsucc = GetNearVertex(Vcurr,opList,closList,["Map";"CloseList"]);    
        for i = 1:length(Vsucc)
            if Vsucc(i).rhs > Vcurr.g+Cost(Vcurr,Vsucc(i),'cityblock')
                Vsucc(i).parent = [Vcurr.coord;Vsucc(i).parent];
                Vsucc(i).rhs = Vcurr.g+Cost(Vcurr,Vsucc(i),'cityblock');
                [opList,closList]=UpdateVetrexs(Vsucc(i),opList,closList);
            end
        end
    elseif Vcurr.rhs > Vcurr.g
        Vcurr.g = inf;
        [opList,closList] = Operate(opList,"UPDATE_VERTEX",Vcurr,closList);     
        Vsucc = GetNearVertex(Vcurr,opList,closList);
        Vsucc = [Vsucc Vcurr]; %论文伪码中有这一行，但是实际运行中Vcurr并没有进行操作，可以根据情况注释掉
        for i = 1:length(Vsucc)
            if Vsucc(i).coord~=Vstart.coord & isParentVertex(Vsucc(i),Vcurr)
                Vsucc(i) = minParentVetrex(Vsucc(i),Vcurr,closList);
                [opList,closList]= UpdateVetrexs(Vsucc(i),opList,closList);
            end
        end
    end
end
end

function Vout = TopKey(opList)
%安全处理，当openList为空的时候就退出ComputeShortestPath()函数
%一般情况下，只有目标节点完全阻塞不能到达导致的
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
%安全处理，当openList为空的时候就退出ComputeShortestPath()函数
%一般情况下，只有目标节点完全阻塞不能到达导致的
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

%获取以TopKey为中心的，周边八个相邻节点进行拓展
%有效的周边节点：没进行初始化的空白节点、已经存放于openList的节点
%无效的周边节点：存在于地图障碍物内的节点、已经存放于closeList的节点
%如果无条件（有效与无效）全部保留，后续的比较操作依然可以把他们（无效节点）去掉。只是涉及到效率问题
function Vsucc = GetNearVertex(Vcurr,opList,closList,ignoreParam)
outFromMap=false;
saveFromClosList=true;
if nargin==4
    for i=1:size(ignoreParam,1)
        switch ignoreParam(i)
            case {"Map","map","MAP"}
                outFromMap=true;
            case {"CloseList","closelist","CLOSELIST"}
                saveFromClosList=false;
        end
    end
else
    ignoreParam=[];
end

global map
xCen = Vcurr.coord(1);
yCen = Vcurr.coord(2);

xCols = [(xCen>1)*(xCen-1),xCen,(xCen<map.size(1))*(xCen+1)];
xCols = xCols(xCols~=0);

yRows = [(yCen>1)*(yCen-1),yCen,(yCen<map.size(2))*(yCen+1)];
yRows = yRows(yRows~=0);

[X,Y] = meshgrid(xCols,yRows);
xyCoord = [X(:),Y(:)];

%对出现在地图障碍物内的Vsucc节点进行处理,true剔除存在于障碍物的Vsucc，false对节点Vsucc进行保留
if outFromMap==true
    i = map.map(xCols,yRows)==1; 
    i=i';
    xyCoord(i,:)=[];
end
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
    elseif res2==true && saveFromClosList==true
        Vsucc = [Vsucc closList(id2)]; %对出现在closList的输出节点Vsucc进行处理，为true保留，为false去除
    end
    %标志位重置
    res1=[];
    res2=[];
end
end

function Vcurr = minParentVetrex(Vcurr,Vold_parent,closList)
Vcurr.parent=Vold_parent.coord;
Vcurr.rhs=Vold_parent.rhs;
closListCoords=vertcat(closList.coord);
if isempty(closListCoords)
    return
end
for i=1:size(Vcurr.parent,1)
        [res,idx]=ismember(Vcurr.parent(i,:),closListCoords,'rows');
        if res==true & Vcurr.rhs > closList(idx).g+Cost(closList(idx),Vcurr,'chebychev')
            Vnew_parent = closList(idx);
            Vcurr.parent = Vnew_parent;
            Vcurr.rhs=Vnew_parent.g+Cost(Vnew_parent,Vcurr,'chebychev');
        end
end
end

function [opList,closList]= UpdateVetrexs(Vcurr,opList,closList)
if isListMember(Vcurr,opList)
    opList = Operate(opList,'UPDATE_KEY',Vcurr,[]);
elseif ~isListMember(Vcurr,opList) && ~isListMember(Vcurr,closList) % && Vcurr.rhs~=Vcurr.g //这里的Vcurr.rhs和Vcurr.g可以视情况注视掉
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
        pushVertList = [pushVertList Vcurr];
        popVertList(idx) = [];
    case 'UPDATE_KEY'
        popVertList(idx)=CalKeyVal(Vcurr);
    case 'UPDATE_VERTEX'
        popVertList(idx)=Vcurr;
        popVertList(idx)=CalKeyVal(Vcurr);

end
end

function result = isListMember(Vcurr,list)
listCoords=vertcat(list.coord);
if ~isempty(listCoords)
    result = ismember(Vcurr.coord,listCoords,'rows');
else
    result = false;
end
end

function result = isParentVertex(Vcurr,Vparent)
parentCoords=vertcat(Vcurr.parent);
result = ismember(Vparent.coord,parentCoords,'rows');
end
