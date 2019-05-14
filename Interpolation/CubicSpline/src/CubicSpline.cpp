#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;
//auto CubicSpline(const double xCoord[],const double yCoord[],const int n,const double dt,const double dP[2]={})->decltype(xCoord)

MatrixXd CubicSpline(const double xCoord[],const double yCoord[],const int n,
                     const double dt,const double dP[2]={})
{
  const VectorXd x = Map<const VectorXd>(xCoord,n);
  const VectorXd y = Map<const VectorXd>(yCoord,n);

  MatrixXd A = MatrixXd::Zero(n,n);
  MatrixXd e = VectorXd::Zero(n,1);
  VectorXd h = VectorXd::Zero(n);

  for(int i=1;i<=n-1;++i)
    h(i-1)=x(i)-x(i-1);

  if(dP!=nullptr)
  {
    double dP1=dP[0];
    double dPn=dP[1];
    A.block<1,2>(0,0)<< 2,1;
    A.block<1,2>(n-1,n-2)<< 1,2;
    e(0)=6*((y(1)-y(0))/h(0)-dP1)/h(0);
    e(n-1)=6*(dPn-(y(n-1)-y(n-2))/h(n-2))/h(n-2);
  }
  else
  {
    A(0,0)=1;
    A(n-1,n-1)=1;
    e(0)=0;
    e(n-1)=0;
  }

  for(int i=1;i<=n-2;++i)
  {
    A.block<1,3>(i,i-1)<< h(i-1),2*(h(i-1)+h(i)),h(i);
    e(i)= 6 * ((y(i+1)-y(i))/h(i) - (y(i)-y(i-1)) / h(i-1));
  }
  MatrixXd M = A.inverse()*e;

  int idxSize=0;
  for(int i =0;i<=n-2;++i)
    idxSize+= VectorXd::LinSpaced(((x(i+1)-x(i))/dt)+1,x(i),x(i+1)).size();
  idxSize-=(n-2);

  MatrixXd pXY(idxSize,2); //   ArrayXd pX(idxSize);
                           //   ArrayXd pY(idxSize);
  int headIdx=0;
  for(int i=0;i<=n-2;++i)
  {
    double step = ((x(i+1)-x(i))/dt)+1;
    ArrayXd px = VectorXd::LinSpaced(step,x(i),x(i+1));
    int k = px.size();

    ArrayXd a(k),b(k),c(k),d(n);
    a=(x(i+1)-px).cube()*M(i)/(6*h(i));
    b=(px-x(i)).cube()*M(i+1)/(6*h(i));
    c=((y(i+1)-y(i))/h(i)-(M(i+1)-M(i))*h(i)/6)*px;
    d(i)=y(i+1) - M(i+1)*h(i)*h(i)/6 - ((y(i+1)-y(i))/h(i) - (M(i+1)-M(i))*h(i)/6)*x(i+1);

    pXY.block(headIdx,1,k,1) = px;         // pX.segment(headIdx,k) = px.col(0);
    pXY.block(headIdx,0,k,1) = a+b+c+d(i); // pY.segment(headIdx,k) = a+b+c+d(i);

    headIdx+=k-1;
  }
//  pXY.col(0)=pX;
//  pXY.col(1)=pY;
  return pXY;
}

using namespace std;
int main(int argc,char** argv)
{
  double x[]={1,2,3,4,5,6,7,8,8.1};
  double y[]={4.5,6.5,1.2,3.4,5.6,4.8,8.8,1.2,1.21};
  double numSize = sizeof(x)/sizeof(*y);
  MatrixXd res = CubicSpline(x,y,numSize,0.01);
//  double dp[2]={0.5,6};
//  MatrixXd res = CubicSpline(x,y,numSize,0.01,dp);
  std::cout<<res<<std::endl;
}
