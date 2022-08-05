/** quartic_polynomial.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Defination of Quartic Polynomial
 */

#ifndef QUARTIC_POLYNOMIAL_H_
#define QUARTIC_POLYNOMIAL_H_

#include <vector>
#include "Eigen/Dense"

namespace fop
{

class QuarticPolynomial
{
 public:
	// Constructor
	QuarticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T);

	// Destructor
	virtual ~QuarticPolynomial() {};
	
	// calculate the s/d coordinate of a point
	double calculatePoint(double t);

	double calculateFirstDerivative(double t);

	double calculateSecondDerivative(double t);

	double calculateThirdDerivative(double t);
	
 private:
	std::vector<double> coefficients;
};

} // namespace fop

#endif //QUARTIC_POLYNOMIAL_H_