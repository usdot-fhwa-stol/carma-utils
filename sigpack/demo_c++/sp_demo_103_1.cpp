#include "sigpack.h"
using namespace arma;
using namespace sp;
int main()
{
    // Set defaults
    rowvec def_vec      = randu<rowvec>(10);
    cx_vec def_vec_cx   = randu<cx_vec>(10);
    mat def_mat         = randu<mat>(5,5);
    cx_mat def_matx     = randu<cx_mat>(5,5);
    Col<int> def_vec_int;
    def_vec_int << 1 << 2 << 3 << 4;

    // Create parser
    sp::parser testpar("..\\test.par");
    
    std::string my_text = testpar.getString("my_text","DEFAULT");     // String
    int               N = testpar.getParam("N",125000);               // Int
    double       pi_val = testpar.getParam("pi_val",3.1);             // Double
    rowvec            x = testpar.getRow("x",def_vec);                // Row vector                
    cx_vec            y = testpar.getCxCol("y",def_vec_cx);           // Col vector complex
    mat               A = testpar.getMat("A",def_mat);                // Mat
    cx_mat            B = testpar.getCxMat("B",def_matx);             // Mat complex
    Col<int>          Z = testpar.getCol("Z",def_vec_int);            // Col integer - false test

    std::cout << "my_text= " << my_text << std::endl;
    std::cout << "N= " << N << std::endl;
    std::cout << "pi_val= " << pi_val << std::endl;
    std::cout << "x= \n" << x << std::endl;
    std::cout << "y= \n" << y << std::endl;
    std::cout << "A= \n" << A << std::endl;
    std::cout << "B= \n" << B << std::endl;
    std::cout << "Z= \n" << Z << std::endl;

    return 0;
}
