#pragma once
#include <iostream>
namespace GEOMETRY {

	enum class NumSolutions {
		zero = 0,
		one,
		two
	};

	struct QuadraticSolution {
		NumSolutions numSolutions{ NumSolutions::zero };
		// I'll use first root as the smallest of the two. Set them both to zero to 
		//begin with
		float firstSolution = 0.0f;
		float secondSolution = 0.0f;
		void print() const { // handy method that prints out the number of solutions and their value(s)
			std::cout << "Number of solutions: " << static_cast<int>(numSolutions) << "\n";
			std::cout << "First solution: " << firstSolution << "\n";
			std::cout << "Second solution: " << secondSolution << "\n";
		}
	};

	QuadraticSolution solveQuadratic(float a, float b, float c);



}
