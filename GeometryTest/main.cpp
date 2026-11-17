//============================================================================
//	Johns Hopkins University Whiting School of Engineering
//	605.667 Principles of Computer Graphics
//	Instructor:	Brian Russin
//
//	Author:  David W. Nesbitt
//	File:    GeometryTest/main.cpp
//	Purpose: Test the vector library.
//
//============================================================================

#include "common/logging.hpp"

#include <stdarg.h>
#include <stdio.h>

namespace cg
{

void vector_test_module1();
void matrix_test_module4();
void vector_test_module5();
void ray_tracer_test();

} // namespace cg

/**
 * Main method. Entry point for application.
 */
int main(int argc, char *argv[])
{
//    cg::init_logging("GeometryTest_Module5.log");
//    cg::vector_test_module5();
    cg::init_logging("RayTracerTest.log");
    cg::ray_tracer_test();
    return 1;
}
