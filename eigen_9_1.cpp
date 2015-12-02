/**helloWorld.cpp*/

//Std
#include <iostream>
#include <math.h>
#include <cstdlib>

//eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define PI 3.14159265

// import most common Eigen types
using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    // Let's print it pretty
    cout.precision(4);

    // Given data
    Vector2f p_O(10.8, -2.7);
    Vector2f m_B(3.1, 1.2);

    double theta = 28.0*PI/180.0;
    double beta = 41.0*PI/180.0;

    Vector2f q_S(12.0, 3.0);

    cout << "The points involved are:" << endl;
    cout << "Coordinates of p wrt O:" << endl;
    cout << p_O << endl << endl;
    cout << "Coordinates of m wrt B:" << endl;
    cout << m_B << endl << endl;
    cout << "Coordinates of q wrt S:" << endl;
    cout << q_S << endl << endl;

    cout << "The rotation matrixes involved are:" << endl;
    Matrix2f R_B_O;
    Matrix2f R_S_B;

    R_B_O << cos(theta), -sin(theta), sin(theta), cos(theta);
    R_S_B << cos(beta), -sin(beta), sin(beta), cos(beta);

    cout << "R of B wrt O:" << endl;
    cout << R_B_O << endl << endl;

    cout << "R of S wrt B:" << endl;
    cout << R_S_B << endl << endl;

    cout << "and the homogeneous matrixes representing the" << endl;
    cout << "transformations from the origin to the base," << endl;
    cout << "and from the base to the sensor, are respectively:" << endl;
    Matrix3f T_B_O;
    Matrix3f T_S_B;

    //Equivalent to T_B_O << R_B_O(0,0), R_B_O(0,1), p_O[0], R_B_O(1,0), R_B_O(1,1), p_O[1], 0, 0, 1;
    T_B_O << R_B_O, p_O, 0, 0, 1;
    //Equivalent to T_S_B << R_S_B(0,0), R_S_B(0,1), m_B[0], R_S_B(1,0), R_S_B(1,1), m_B[1], 0, 0, 1;
    T_S_B << R_S_B, m_B, 0, 0, 1;

    cout << "T of B wrt O:" << endl;
    cout << T_B_O << endl << endl;

    cout << "T of S wrt B:" << endl;
    cout << T_S_B << endl << endl;

    cout << "So the point q with respect to the vehicle base frame" << endl;
    cout << "and with respect to the origin of the trajectory is:" << endl;

    Vector3f q_S_tmp(q_S[0], q_S[1], 1);

    Vector3f q_B_tmp = T_S_B * q_S_tmp;
    Vector3f q_O_tmp = T_B_O * T_S_B * q_S_tmp;

    Vector2f q_B(q_B_tmp[0], q_B_tmp[1]);
    Vector2f q_O(q_O_tmp[0], q_O_tmp[1]);

    cout << "Coordinates of q wrt B:" << endl;
    cout << q_B << endl << endl;

    cout << "Coordinates of q wrt O:" << endl;
    cout << q_O << endl << endl;

    return 0;
}
