//
// Copyright (c) 2017 CNRS
// Authors: Mathieu Geisert
//
// This file is part of hpp-rbprm
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_LEARNING_GMM_HH
#define HPP_RBPRM_LEARNING_GMM_HH

//Gaussian Mixture Model taken from "locomote" library of Justin Carpentier.
#include "locomote/stats/gmm-full.hpp"

namespace hpp {
  namespace rbprm {


    typedef locomote::stats::GaussianMixtureModelFull<double, 9> GMM;
    typedef boost::shared_ptr <GMM> GMMPtr_t;

    typedef GMM::VectorCovarianceMatrix VectorCovarianceMatrix;
    typedef GMM::MatrixDx MatrixDx;
    typedef GMM::MatrixD MatrixD;
    typedef GMM::VectorX VectorX;


  } // namesapce rbprm
} // namespace hpp
#endif // HPP_RBPRM_LEARNING_GMM_HH
