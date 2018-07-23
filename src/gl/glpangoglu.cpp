/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pangolin/gl/glpangoglu.h>
#include <pangolin/utils/simple_math.h>

#include <vector>

#ifdef HAVE_EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

namespace pangolin {

const GLubyte gNotErrorLookup[] = "XX";

const GLubyte* glErrorString(GLenum /*error*/)
{
    // TODO: Implement glErrorString
    return gNotErrorLookup;
}

// Based on glu implementation.
template<typename P>
int InvertMatrix(const P m[16], P invOut[16])
{
    P inv[16], det;
    int i;

    inv[0] =   m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15]
             + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4] =  -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15]
             - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8] =   m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15]
             + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14]
             - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1] =  -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15]
             - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5] =   m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15]
             + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9] =  -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15]
             - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] =  m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14]
             + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2] =   m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15]
             + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6] =  -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15]
             - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] =  m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15]
             + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14]
             - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3] =  -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11]
             - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7] =   m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11]
             + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11]
             - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] =  m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10]
             + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];

    det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (det == 0)
        return GL_FALSE;

    det=1.0f/det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return GL_TRUE;
}

// Based on glu implementation
GLint glProject(
    float objx, float objy, float objz,
    const float modelMatrix[16],
    const float projMatrix[16],
    const GLint viewport[4],
    float* winx, float* winy, float* winz)
{
    float t1[4] = {objx, objy, objz, 1.0f};
    float t2[4];

    MatMul<4,4,1,float>(t2, modelMatrix, t1);
    MatMul<4,4,1,float>(t1, projMatrix, t2);

    if (t1[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    t1[0]/=t1[3];
    t1[1]/=t1[3];
    t1[2]/=t1[3];

    // Map x, y and z to range 0-1
    t1[0]=t1[0]*0.5f+0.5f;
    t1[1]=t1[1]*0.5f+0.5f;
    t1[2]=t1[2]*0.5f+0.5f;

    // Map x,y to viewport
    t1[0]=t1[0] * viewport[2] + viewport[0];
    t1[1]=t1[1] * viewport[3] + viewport[1];

    *winx=t1[0];
    *winy=t1[1];
    *winz=t1[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glUnProject(
    float winx, float winy, float winz,
    const float mv[16],
    const float proj[16],
    const GLint viewport[4],
    float* objx, float* objy, float* objz)
{
    float t1[16];

    MatMul<4,4,4,float>(t1, proj, mv);

    if (!InvertMatrix<float>(t1, t1)) {
        return(GL_FALSE);
    }

    // Map x and y from window coordinates
    float in[4] = {winx, winy, winz, 1.0f};
    in[0] = (in[0] - viewport[0]) / viewport[2];
    in[1] = (in[1] - viewport[1]) / viewport[3];

    // Map to range -1 to 1
    in[0] = in[0] * 2 - 1;
    in[1] = in[1] * 2 - 1;
    in[2] = in[2] * 2 - 1;

    float out[4];
    MatMul<4,4,1,float>(out, t1, in);

    if (out[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    out[0] /= out[3];
    out[1] /= out[3];
    out[2] /= out[3];

    // Copy out
    *objx = out[0];
    *objy = out[1];
    *objz = out[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glProject(
    double objx, double objy, double objz,
    const double modelMatrix[16],
    const double projMatrix[16],
    const GLint viewport[4],
    double* winx, double* winy, double* winz)
{
    double t1[4] = {objx, objy, objz, 1.0f};
    double t2[4];

    MatMul<4,4,1,double>(t2, modelMatrix, t1);
    MatMul<4,4,1,double>(t1, projMatrix, t2);

    if (t1[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    t1[0]/=t1[3];
    t1[1]/=t1[3];
    t1[2]/=t1[3];

    // Map x, y and z to range 0-1
    t1[0]=t1[0]*0.5f+0.5f;
    t1[1]=t1[1]*0.5f+0.5f;
    t1[2]=t1[2]*0.5f+0.5f;

    // Map x,y to viewport
    t1[0]=t1[0] * viewport[2] + viewport[0];
    t1[1]=t1[1] * viewport[3] + viewport[1];

    *winx=t1[0];
    *winy=t1[1];
    *winz=t1[2];

    return GL_TRUE;
}

// Based on glu implementation
GLint glUnProject(
    double winx, double winy, double winz,
    const double mv[16],
    const double proj[16],
    const GLint viewport[4],
    double* objx, double* objy, double* objz)
{
    double t1[16];

    MatMul<4,4,4,double>(t1, proj, mv);

    if (!InvertMatrix<double>(t1, t1)) {
        return(GL_FALSE);
    }

    // Map x and y from window coordinates
    double in[4] = {winx, winy, winz, 1.0f};
    in[0] = (in[0] - viewport[0]) / viewport[2];
    in[1] = (in[1] - viewport[1]) / viewport[3];

    // Map to range -1 to 1
    in[0] = in[0] * 2 - 1;
    in[1] = in[1] * 2 - 1;
    in[2] = in[2] * 2 - 1;

    double out[4];
    MatMul<4,4,1,double>(out, t1, in);

    if (out[3] == 0.0) {
        return(GL_FALSE);
    }

    // Normalise
    out[0] /= out[3];
    out[1] /= out[3];
    out[2] /= out[3];

    // Copy out
    *objx = out[0];
    *objy = out[1];
    *objz = out[2];

    return GL_TRUE;
}

#ifdef HAVE_EIGEN
GLint glUnProjectList(
    std::vector<float> winx, std::vector<float> winy, std::vector<float> winz,
    const double mv[16],
    const double proj[16],
    const GLint viewport[4],
    Eigen::VectorXd& objx, Eigen::VectorXd& objy, Eigen::VectorXd& objz)
{
    double t1[16];

    MatMul<4,4,4,double>(t1, proj, mv);

    if (!InvertMatrix<double>(t1, t1)) {
        return(GL_FALSE);
    }

    Eigen::Map<Eigen::RowVectorXf> winglx(winx.data(), winx.size());
    Eigen::Map<Eigen::RowVectorXf> wingly(winy.data(), winy.size());
    Eigen::Map<Eigen::RowVectorXf> winglz(winz.data(), winz.size());

    // Map x and y from window coordinates
//    double in[4] = {winx, winy, winz, 1.0f};
//    in[0] = (in[0] - viewport[0]) / viewport[2];
//    in[1] = (in[1] - viewport[1]) / viewport[3];

    // Map to range -1 to 1
//    in[0] = in[0] * 2 - 1;
//    in[1] = in[1] * 2 - 1;
//    in[2] = in[2] * 2 - 1;

    // Map x and y from window coordinates
    // Map to range -1 to 1
    Eigen::VectorXf in0 = ((winglx.array()-viewport[0]) / viewport[2] * 2) - 1;
    Eigen::VectorXf in1 = ((wingly.array()-viewport[1]) / viewport[3] * 2) - 1;
    Eigen::VectorXf in2 = (winglz.array() * 2) - 1;

//    double out[4];
//    MatMul<4,4,1,double>(out, t1, in);

    Eigen::Matrix4Xf in;
    in.resize(Eigen::NoChange, winglz.size());
    //in << in0, in1, in2, Eigen::VectorXf::Ones(winglx.size());
    in.row(0) = in0;
    in.row(1) = in1;
    in.row(2) = in2;
    in.row(3) = Eigen::VectorXf::Ones(winglx.size());
//    Eigen::Matrix3Xf in;
//    in.resize(Eigen::NoChange, winglz.size());
//    in << in0, in1, in2;
    Eigen::Map<Eigen::Matrix4d> tt1(t1);
//    Eigen::Affine3d ttt1((Eigen::Map<Eigen::Matrix4d>(t1)));

//    for(uint i=0; i<in.cols(); i++) {
//        std::cout << i << ": " << in.col(i).transpose() << std::endl;
//    }

//    std::cout << in.rows() << ", " << in.cols() << std::endl;
//    std::cout << "min " << in.rowwise().minCoeff().transpose() << std::endl;
//    std::cout << "max " << in.rowwise().maxCoeff().transpose() << std::endl;

    //auto outout = ttt1.matrix().array() * in.array().colwise();
    // https://stackoverflow.com/a/38845543/8144672
    //Eigen::MatrixXd outout = (ttt1.linear() * in.cast<double>()) + ttt1.translation();
//    Eigen::MatrixXd outout = ttt1.linear() * in.cast<double>();
    Eigen::Matrix4Xd outout(4, winz.size());

//    std::cout << "tt1 " << std::endl << tt1 << std::endl;
//    std::cout << "in " << std::endl << in.cast<double>().col(0).transpose() << std::endl;

    for(uint i=0; i<winz.size(); i++) {
        if(winz[i]<1.0f) {
//            Eigen::Vector4d aa = in.cast<double>().col(i);
//            std::cout << i << " aa " << aa.transpose() << std::endl;
//            Eigen::Vector4d bb = tt1 * aa;
//            std::cout << i << " bb " << bb.transpose() << std::endl;
//            outout.col(i) = bb / bb[3];
//            std::cout << i << " oo " << outout.col(i).transpose() << std::endl;
            outout.col(i) = tt1 * in.cast<double>().col(i);
        }
    }

//    std::cout << "out " << std::endl << outout.col(0).transpose() << std::endl;

//    std::cout << outout.rows() << ", " << outout.cols() << std::endl;
//    std::cout << "min " << outout.rowwise().minCoeff().transpose() << std::endl;
//    std::cout << "max " << outout.rowwise().maxCoeff().transpose() << std::endl;

//    Eigen::Map<Eigen::Matrix4d> tt1(t1);
//    auto outout = tt1 * in.colwise();

//    if (out[3] == 0.0) {
//        return(GL_FALSE);
//    }

    // Normalise
//    out[0] /= out[3];
//    out[1] /= out[3];
//    out[2] /= out[3];

    // Copy out
//    *objx = out[0];
//    *objy = out[1];
//    *objz = out[2];

//    const Eigen::VectorXd &outx = outout.row(0).array();
//    const Eigen::VectorXd &outy = outout.row(1).array();
//    const Eigen::VectorXd &outz = outout.row(2).array();

    const Eigen::VectorXd &outx = outout.row(0).array() / outout.row(3).array();
    const Eigen::VectorXd &outy = outout.row(1).array() / outout.row(3).array();
    const Eigen::VectorXd &outz = outout.row(2).array() / outout.row(3).array();

//    std::cout << "outout min " << outout.rowwise().minCoeff().transpose() << std::endl;
//    std::cout << "outout max " << outout.rowwise().maxCoeff().transpose() << std::endl;

//    objx->assign(outx.data(), outx.data()+outx.size());
//    objy->assign(outy.data(), outy.data()+outy.size());
//    objz->assign(outz.data(), outz.data()+outz.size());
//    *objx = std::vector<double>(outx.data(), outx.data()+outx.size());
//    *objy = std::vector<double>(outy.data(), outy.data()+outy.size());
//    *objz = std::vector<double>(outz.data(), outz.data()+outz.size());

//    std::cout << *std::min_element(objx->begin(), objx->end()) << ", " << *std::max_element(objx->begin(), objx->end()) << std::endl;

    objx = outx;
    objy = outy;
    objz = outz;

    return GL_TRUE;
}
#endif

void glUnProjectList2(
    std::vector<float> winx, std::vector<float> winy, std::vector<float> winz,
    const double mv[16],
    const double proj[16],
    const GLint viewport[4],
    std::vector<double>* objx, std::vector<double>* objy, std::vector<double>* objz)
{
    if(winx.size()!=winz.size() || winy.size()!=winz.size())
        throw std::runtime_error("coordinate dimensions mismatch!");

    double t1[16];

    MatMul<4,4,4,double>(t1, proj, mv);

    if (!InvertMatrix<double>(t1, t1)) {
        return;
    }

    objx->resize(winx.size());
    objy->resize(winy.size());
    objz->resize(winz.size());

    for(uint i=0; i<winz.size(); i++) {
        // Map x and y from window coordinates
        double in[4] = {winx[i], winy[i], winz[i], 1.0f};
        in[0] = (in[0] - viewport[0]) / viewport[2];
        in[1] = (in[1] - viewport[1]) / viewport[3];

        // Map to range -1 to 1
        in[0] = in[0] * 2 - 1;
        in[1] = in[1] * 2 - 1;
        in[2] = in[2] * 2 - 1;

        double out[4];
        MatMul<4,4,1,double>(out, t1, in);

        // Normalise
        out[0] /= out[3];
        out[1] /= out[3];
        out[2] /= out[3];

        // Copy out
        (*objx)[i] = out[0];
        (*objy)[i] = out[1];
        (*objz)[i] = out[2];
    }
}


}
