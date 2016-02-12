#ifndef __EIGENMULTIVARIATENORMAL_HPP
#define __EIGENMULTIVARIATENORMAL_HPP

#include <Eigen/Dense>

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

namespace shared {

class MultivariateNormalBase {
	
};

/**
    We find the eigen-decomposition of the covariance matrix.
    We create a vector of normal samples scaled by the eigenvalues.
    We rotate the vector by the eigenvectors.
    We add the mean.
*/
template<typename _Scalar>
class EigenMultivariateNormal: public MultivariateNormalBase
{
    //boost::mt19937 rng;    // The uniform pseudo-random algorithm
    boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
       randN; // The 0-mean unit-variance normal generator

    Eigen::MatrixXd rot;
    Eigen::MatrixXd scl;

    Eigen::MatrixXd mean;

public:
    EigenMultivariateNormal(boost::mt19937 &generator)
        : randN(generator ,norm)
    {
        
    }

    void setCovar(Eigen::MatrixXd& covarMat)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd >
           eigenSolver(covarMat);
        rot = eigenSolver.eigenvectors();
        scl = eigenSolver.eigenvalues();
        for (int ii=0; ii < covarMat.rows(); ++ii) {
            scl(ii,0) = sqrt(scl(ii,0));
        }
    }

    void setMean(Eigen::MatrixXd& meanVec)
    {
        mean = meanVec;
    }

    void nextSample(Eigen::MatrixXd& sampleVec)
    {
        for (int ii=0; ii < sampleVec.rows(); ++ii) {
            sampleVec(ii,0) = randN()*scl(ii,0);
        }
        sampleVec = rot*sampleVec + mean;
    }
    
};

}

#endif