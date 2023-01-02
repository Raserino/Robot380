#include <BasicLinearAlgebra.h>
const double l0 = 1;
const double l1 = 1, b1 = 0;
const double l2 = 0, b2 = 0;
const double l3 = 1, b3 = 0;
const double l4 = 1;
int state = 0;

BLA::Matrix<3> forward(BLA::Matrix<3> theta)
{
  /*
   * Jonas Nebl - 17-Sep-2022
   * Forward transformation for the robot arm
   * Tested and working 
   */
  //input joint values
  double t1 = theta(0);
  double t2 = theta(1);
  double t3 = theta(2);
  double t4 = 3.14159/2-t2-t3;
  
  BLA::Matrix<4,4> T_01;
  T_01.Fill(0);
  T_01(0,0) = cos(t1);
  T_01(0,1) = -sin(t1);
  T_01(1,0) = sin(t1);
  T_01(1,1) = cos(t1);
  T_01(2,2) = 1;
  T_01(2,3) = l0;
  T_01(3,3) = 1;
  BLA::Matrix<4,4> T_12;
  T_12.Fill(0);
  T_12(0,0) = cos(t2);
  T_12(0,2) = sin(t2);
  T_12(1,1) = 1;
  T_12(2,0) = -sin(t2);
  T_12(2,2) = cos(t2);
  T_12(2,3) = l1;
  T_12(3,3) = 1;
  BLA::Matrix<4,4> T_23;
  T_23.Fill(0);
  T_23(0,0) = cos(t3);
  T_23(0,2) = sin(t3);
  T_23(0,3) = l2;
  T_23(1,1) = 1;
  T_23(2,0) = -sin(t3);
  T_23(2,2) = cos(t3);
  T_23(3,3) = 1;
  BLA::Matrix<4,4> T_34;
  T_34.Fill(0);
  T_34(0,0) = cos(t4);
  T_34(0,2) = sin(t4);
  T_34(0,3) = l3;
  T_34(1,1) = 1;
  T_34(2,0) = -sin(t4);
  T_34(2,2) = cos(t4);
  T_34(3,3) = 1;
  
  BLA::Matrix<4> r4;
  r4.Fill(0);
  r4(0) = l4;
  r4(3) = 1;
  BLA::Matrix<4> X = T_01*T_12*T_23*T_34*r4;
  
  BLA::Matrix<3> X_return;
  for(int i=0; i<3; i++)
  {
    X_return(i) = X(i);
  }
  return X_return;
}

BLA::Matrix<3,3> Jacobian(BLA::Matrix<3> theta)
{
  BLA::Matrix<3,3> J;
  const double h = 0.005;
  
  for (int j=0; j<3; j++)
  {
    BLA::Matrix<3> theta_plush = theta;
    theta_plush(j) += h;
    BLA::Matrix<3> forward_plush = forward(theta_plush);
    BLA::Matrix<3> theta_minush = theta;
    theta_minush(j) -= h;
    BLA::Matrix<3> forward_minush = forward(theta_minush);
    for(int i=0; i<3; i++)
    {
      J(i,j) = (forward_plush(i)-forward_minush(i))/(2*h);
    }
  }
  return J;
}

BLA::Matrix<3> backward(BLA::Matrix<3> X_d)
{
  const double tol = 0.01;
  BLA::Matrix<3> theta_new = {0.5,0.5,0.5};
  double err = 1000;
  BLA::Matrix<3,3> J = Jacobian(theta_new);
  int iter = 0;
  
  do
  {   
    BLA::Matrix<3> theta = theta_new;
    BLA::Matrix<3,3> J = Jacobian(theta);
    BLA::Matrix<3,3> J_decomp = J; //LUDecompose destroys J_decomp
    auto decomp = LUDecompose(J_decomp);
    BLA::Matrix<3> X_lusolve = LUSolve(decomp, forward(theta)-X_d); 
    theta_new = theta - X_lusolve;
    BLA::Matrix<3> diff = theta_new - theta;
    err = sqrt(pow(diff(0),2) + pow(diff(1),2) + pow(diff(2),2));
    iter++;
  } 
  while(err > tol && iter < 30);
  return theta_new;
}

double rad2deg(double rad)
{
  return rad*360/(2*3.14159);
}

double deg2rad(double deg)
{
  return deg*(2*3.14159)/360;
}

void printMatrix(BLA::Matrix<3,3> matrix)
{ 
  for(int i = 0; i<3; i++)
  {
    for(int j = 0; j<3; j++)
    {
      Serial.print(matrix(i,j),6);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
  return;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Start");

  long t_start = millis();
  BLA::Matrix<3> X_d= {1, 1, 1};
  BLA::Matrix<3> theta = backward(X_d);
  long duration = millis() - t_start;
  
  Serial.print(rad2deg(theta(0)));
  Serial.print(" ");
  Serial.print(rad2deg(theta(1)));
  Serial.print(" ");
  Serial.print(rad2deg(theta(2)));
  Serial.print(" time: ");
  Serial.print(duration);
  Serial.print('\n');
  
  delay(1000);
  
//  switch(state)
//  {
//    case 0:
//      break;
//    default:
//      break;
//  } 
}

void loop()
{
  
}
