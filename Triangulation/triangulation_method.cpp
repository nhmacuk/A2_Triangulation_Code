/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "triangulation.h"
#include "matrix_algo.h"
#include <easy3d/optimizer/optimizer_lm.h>


using namespace easy3d;


/**
 * TODO: Finish this function for reconstructing 3D geometry from corresponding image points.
 * @return True on success, otherwise false. On success, the reconstructed 3D points must be written to 'points_3d'
 *      and the recovered relative pose must be written to R and t.
 */
bool Triangulation::triangulation(
        double fx, double fy,     /// input: the focal lengths (same for both cameras)
        double cx, double cy,     /// input: the principal point (same for both cameras)
        const std::vector<Vector2D> &points_0,  /// input: 2D image points in the 1st image.
        const std::vector<Vector2D> &points_1,  /// input: 2D image points in the 2nd image.
        std::vector<Vector3D> &points_3d,       /// output: reconstructed 3D points
        Matrix33 &R,   /// output: 3 by 3 matrix, which is the recovered rotation of the 2nd camera
        Vector3D &t    /// output: 3D vector, which is the recovered translation of the 2nd camera
) const
{
    /// NOTE: there might be multiple workflows for reconstructing 3D geometry from corresponding image points.
    ///       This assignment uses the commonly used one explained in our lecture.
    ///       It is advised to define a function for the sub-tasks. This way you have a clean and well-structured
    ///       implementation, which also makes testing and debugging easier. You can put your other functions above
    ///       triangulation(), or put them in one or multiple separate files.

//    std::cout << "\nTODO: I am going to implement the triangulation() function in the following file:" << std::endl
//              << "\t    - triangulation_method.cpp\n\n";

    /*
    std::cout << "[Liangliang]:\n"
                 "\tFeel free to use any provided data structures and functions. For your convenience, the\n"
                 "\tfollowing three files implement basic linear algebra data structures and operations:\n"
                 "\t    - Triangulation/matrix.h  Matrices of arbitrary dimensions and related functions.\n"
                 "\t    - Triangulation/vector.h  Vectors of arbitrary dimensions and related functions.\n"
                 "\t    - Triangulation/matrix_algo.h  Determinant, inverse, SVD, linear least-squares...\n"
                 "\tPlease refer to the above files for a complete list of useful functions and their usage.\n\n"
                 "\tIf you choose to implement the non-linear method for triangulation (optional task). Please\n"
                 "\trefer to 'Tutorial_NonlinearLeastSquares/main.cpp' for an example and some explanations.\n\n"
                 "\tIn your final submission, please\n"
                 "\t    - delete ALL unrelated test or debug code and avoid unnecessary output.\n"
                 "\t    - include all the source code (and please do NOT modify the structure of the directories).\n"
                 "\t    - do NOT include the 'build' directory (which contains the intermediate files in a build step).\n"
                 "\t    - make sure your code compiles and can reproduce your results without ANY modification.\n\n" << std::flush;
    */

    /*
    /// Below are a few examples showing some useful data structures and APIs.

    /// define a 2D vector/point
    Vector2D b(1.1, 2.2);

    /// define a 3D vector/point
    Vector3D a(1.1, 2.2, 3.3);

    /// get the Cartesian coordinates of a (a is treated as Homogeneous coordinates)
    Vector2D p = a.cartesian();

    /// get the Homogeneous coordinates of p
    Vector3D q = p.homogeneous();

    /// define a 3 by 3 matrix (and all elements initialized to 0.0)
    Matrix33 A;

    /// define and initialize a 3 by 3 matrix
    Matrix33 T(1.1, 2.2, 3.3,
               0, 2.2, 3.3,
               0, 0, 1);

    /// define and initialize a 3 by 4 matrix
    Matrix34 M(1.1, 2.2, 3.3, 0,
               0, 2.2, 3.3, 1,
               0, 0, 1, 1);

    /// set first row by a vector
    M.set_row(0, Vector4D(1.1, 2.2, 3.3, 4.4));

    /// set second column by a vector
    M.set_column(1, Vector3D(5.5, 5.5, 5.5));

    /// define a 15 by 9 matrix (and all elements initialized to 0.0)
    Matrix W(15, 9, 0.0);
    /// set the first row by a 9-dimensional vector
    W.set_row(0, {0, 1, 2, 3, 4, 5, 6, 7, 8}); // {....} is equivalent to a std::vector<double>

    /// get the number of rows.
    int num_rows = W.rows();

    /// get the number of columns.
    int num_cols = W.cols();

    /// get the the element at row 1 and column 2
    double value = W(1, 2);

    /// get the last column of a matrix
    Vector last_column = W.get_column(W.cols() - 1);

    /// define a 3 by 3 identity matrix
    Matrix33 I = Matrix::identity(3, 3, 1.0);

    /// matrix-vector product
    Vector3D v = M * Vector4D(1, 2, 3, 4); // M is 3 by 4

    ///For more functions of Matrix and Vector, please refer to 'matrix.h' and 'vector.h'

    // TODO: delete all above example code in your final submission
    */
    //--------------------------------------------------------------------------------------------------------------
    // implementation starts ...

//    double fx, double fy,     /// input: the focal lengths (same for both cameras)
//    double cx, double cy,     /// input: the principal point (same for both cameras)
//    const std::vector<Vector2D> &points_0,  /// input: 2D image points in the 1st image.
//    const std::vector<Vector2D> &points_1,  /// input: 2D image points in the 2nd image.

    // TODO: check if the input is valid (always good because you never known how others will call your function).

    // TODO: Estimate relative pose of two views. This can be subdivided into
    //      - estimate the fundamental matrix F;
    //      - compute the essential matrix E;
    //      - recover rotation R and t.
    Matrix33 K(fx, 0, cx, //intrinsic camera matrix K
               0, fy, cy,
               0, 0, 1);
    std::cout << "2D points size:" << points_0.size() << "\n";

    // first estimate:
    std::vector<double> random_draws = {6, 17, 39, 70, 84, 100, 116, 151};
    Vector Ran(random_draws);

    // normalisation for point0
    double sumx=0.0;
    double sumy=0.0;
    for (int i = 0; i < random_draws.size(); i++) {
        sumx=sumx+points_0[Ran[i]][0];
        sumy=sumy+points_0[Ran[i]][1];
    }
    double centerx = sumx/8;
    double centery = sumy/8;

    Vector DistanceFromCenter(8, 0);
    double sumdistance=0;
    for (int i = 0; i < DistanceFromCenter.size(); i++) {
        double x = points_0[i][0];
        double y = points_0[i][1];
        DistanceFromCenter[i]= sqrt((centerx-x)*(centerx-x)+(centery-y)*(centery-y));
        sumdistance=sumdistance+DistanceFromCenter[i];
    }

    double Avgdistance=sumdistance/DistanceFromCenter.size();
    double scale=sqrt(2)/Avgdistance;
    Vector NormalizedDistance=DistanceFromCenter*scale;

    // normalisation for point1
    double sumx1=0.0;
    double sumy1=0.0;
    for (int i = 0; i < random_draws.size(); i++) {
        sumx1=sumx1+points_1[Ran[i]][0];
        sumy1=sumy1+points_1[Ran[i]][1];
    }
    double centerx1 = sumx1/8;
    double centery1 = sumy1/8;

    Vector DistanceFromCenter1(8, 0);
    double sumdistance1=0;
    for (int i = 0; i < DistanceFromCenter1.size(); i++) {
        double x1 = points_1[i][0];
        double y1 = points_1[i][1];
        DistanceFromCenter1[i]= sqrt((centerx1-x1)*(centerx1-x1)+(centery1-y1)*(centery1-y1));
        sumdistance1=sumdistance1+DistanceFromCenter1[i];
    }

    double Avgdistance1=sumdistance1/DistanceFromCenter1.size();
    double scale1=sqrt(2)/Avgdistance1;
    Vector NormalizedDistance1=DistanceFromCenter1*scale1;

//    for (int i = 0; i<NormalizedDistance1.size(); i++) {
//        std::cout << NormalizedDistance1[i] << "\t";
//    }
//    std::cout << "\n";

    // Create W
    Matrix W(8, 9, 0.0);
    for (int i = 0; i < W.rows(); i++) {
        W.set_row(i,{new_points_0[Ran[i]][0]*new_points_1[Ran[i]][0], new_points_0[Ran[i]][1]*new_points_1[Ran[i]][0], new_points_1[Ran[i]][0], new_points_0[Ran[i]][0]*new_points_1[Ran[i]][1], new_points_0[Ran[i]][1]*new_points_1[Ran[i]][1], new_points_1[Ran[i]][1], new_points_0[Ran[i]][0], new_points_0[Ran[i]][1],1});

//        W.set_row(i,{points_0[Ran[i]][0]*points_1[Ran[i]][0], points_0[Ran[i]][1]*points_1[Ran[i]][0], points_1[Ran[i]][0], points_0[Ran[i]][0]*points_1[Ran[i]][1], points_0[Ran[i]][1]*points_1[Ran[i]][1], points_1[Ran[i]][1], points_0[Ran[i]][0], points_0[Ran[i]][1],1});

//        W.set_row(i,{NormalizedDistance[i]*NormalizedDistance1[i], NormalizedDistance[i]*NormalizedDistance1[i], NormalizedDistance1[i], NormalizedDistance[i]*NormalizedDistance1[i], NormalizedDistance[i]*NormalizedDistance1[i], NormalizedDistance1[i], NormalizedDistance[i], NormalizedDistance[i],1});
    }

//    std::cout << W << "\n";

    // Compute the SVD decomposition of A

    /// matrix-vector product
    int mm = W.rows(); int nn = W.cols();
    Matrix U(mm, mm, 0.0);   // initialized with 0s
    Matrix S(mm, nn, 0.0);   // Sigma matrix
    Matrix V(nn, nn, 0.0);   // initialized with 0s

    svd_decompose(W, U, S, V);
    std::cout << "my matrix S: " << S;

    // F^
    Vector F_hat_vector= (V.get_column(V.cols() - 1));

    // F from 1x9 to 3x3
    Matrix F_hat(3,3, {F_hat_vector[0], F_hat_vector[1], F_hat_vector[2],
                       F_hat_vector[3], F_hat_vector[4], F_hat_vector[5],
                       F_hat_vector[6], F_hat_vector[7], F_hat_vector[8],});

    mm = F_hat.rows(); nn = F_hat.cols();
    Matrix U1(mm, mm, 0.0);   // initialized with 0s
    Matrix S1(mm, nn, 0.0);   // Sigma matrix
    Matrix V1(nn, nn, 0.0);   // initialized with 0s

    svd_decompose(F_hat, U1, S1, V1);

    std::cout << "my matrix S1: " << S1;

//     Create Sigma
//    std::vector<double> myvector = {S1[0][0], 0, 0, 0, S1[1][1], 0, 0, 0, 0};
//    Matrix Sigma(3,3, myvector);
//    std::cout << "my matrix Simga: " << Sigma;

// Create F
//    Matrix F;
//    F = U1 * Sigma * transpose(V1);  //This is F_g

    //  denormalize
    Matrix F;
    F = T1.transpose()*F_hat*T;
    std::cout << "my matrix F: " << F; //no no no I was wrong

    Matrix E = transpose(K)*F*K;
    std::cout << "my matrix E: " << E;
    //    // check if R is valid
    //    std::cout << "R.T*R " << R * transpose(R) << "\n";



    // TODO: Reconstruct 3D points. The main task is
    //      - triangulate a pair of image points (i.e., compute the 3D coordinates for each corresponding point pair)

    // TODO: Don't forget to
    //          - write your recovered 3D points into 'points_3d' (so the viewer can visualize the 3D points for you);
    //          - write the recovered relative pose into R and t (the view will be updated as seen from the 2nd camera,
    //            which can help you check if R and t are correct).
    //       You must return either 'true' or 'false' to indicate whether the triangulation was successful (so the
    //       viewer will be notified to visualize the 3D points and update the view).
    //       There are a few cases you should return 'false' instead, for example:
    //          - function not implemented yet;
    //          - input not valid (e.g., not enough points, point numbers don't match);
    //          - encountered failure in any step.
    return points_3d.size() > 0;
}