#include "QuadraticSolver.h"
#include "Vector.h"
#include <algorithm>

using namespace GEOMETRY;

QuadraticSolution GEOMETRY::solveQuadratic(float a, float b, float c)
{

	QuadraticSolution answer;
	float discriminant = b * b - 4.0f * a * c;
	// TODO for assignment 2

	if (discriminant < 0) {
		return answer; // return a default QuadraticSolution
	}
	else if (discriminant < VERY_SMALL) {
		// if discriminant is zero
		// then there is one solution.
		// set both solutions (firstSolution and secondSolution) to -b/2a
		answer.firstSolution = -b / (2.0f * a);
		answer.secondSolution = answer.firstSolution;
		answer.numSolutions = NumSolutions::one;
	}
	else {
		// lastly, if discriminant is greater than zero
		// then there are two solutions.
		// set firstSolution to (-b - sqrt(discriminant)) / 2a
		// set secondSolution to (-b + sqrt(discriminant)) / 2a
		// It is useful to make sure the first solution is the smaller of the two
		float x1 = (-b + sqrt(discriminant)) / (2.0f * a);
		float x2 = (-b - sqrt(discriminant)) / (2.0f * a);
		// I want the first solution to be the smallest of the two
		answer.firstSolution = std::min(x1, x2);
		answer.secondSolution = std::max(x1, x2);
		answer.numSolutions = NumSolutions::two;
	}
	return answer;
}
