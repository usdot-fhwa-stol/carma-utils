#include "sigpack.h"

using namespace std;
using namespace arma;
using namespace sp;

int main()
{
    int N = 100;
    
	vec X_vec = linspace(-2,2,N); 
	vec Y_vec(N,fill::randn); 
	mat A_mat(N,N,fill::randn);
	mat B_mat(5,N,fill::randn);
	cube C_cube(N,N,3,fill::randu);
		
    gplot gp0;
    gp0.window("Plot1", 10,  10,  500, 520);

    //========================================
    gp0.title("Multiplot Y - no label");
    gp0.plot_add_mat(B_mat);
    gp0.plot_show();    
    cin.ignore();  
    
    //========================================
    gp0.title("Multiplot Y - label");
    gp0.plot_add_mat(B_mat,"Data-");
    gp0.plot_show();    
    cin.ignore();  
    
    //========================================
    gp0.title("Multiplot Y vs X - label");
    gp0.plot_add(X_vec,B_mat.row(0),"Data-0");
    gp0.plot_add(X_vec,B_mat.row(1),"Data-1");
    gp0.plot_add(X_vec,B_mat.row(2),"Data-2");
    gp0.plot_add(X_vec,B_mat.row(3),"Data-3");
    gp0.plot_show();    
    cin.ignore();  

   //========================================
    gp0.title("Y - no label");
    gp0.plot_add(Y_vec,"");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Y - label");
    gp0.plot_add(Y_vec,"Y data");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Y vs x - no label");
    gp0.plot_add(X_vec,Y_vec,"");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Y vs x - label");
    gp0.plot_add(X_vec,Y_vec,"Y vs X");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Y vs x - label + X/Y label");
    gp0.xlabel("X data");
    gp0.ylabel("Y data");
    gp0.plot_add(X_vec,Y_vec,"Y vs X");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Y vs x - X,YLim");
    gp0.xlabel("X data");
    gp0.ylabel("Y data");
    gp0.label(0,0.3,"x=0,y=0.3");
    gp0.xlim(-0.1,0.4);
    gp0.ylim(-1,1);
    gp0.plot_add(X_vec,Y_vec,"Y vs X");
    gp0.plot_show();    
    cin.ignore();  
    
    gp0.reset_term();
    //========================================
    gp0.title("Scatter");
    vec sc_y=X_vec+Y_vec/5;
    gp0.plot_add(X_vec,sc_y,"","points");
    gp0.plot_show();    
    cin.ignore();  

    //========================================
    gp0.title("Scatter - 2");
    gp0.plot_add(X_vec,sc_y,"Data 1","points");
    sc_y=-X_vec+Y_vec/10;
    gp0.plot_add(X_vec,sc_y,"Data 2","points");
    gp0.plot_show();    
    cin.ignore();  
    
    gp0.send2gp("reset");
    //========================================
    gp0.title("Mesh");
    gp0.mesh(A_mat);
    cin.ignore();  

    gp0.send2gp("reset");
    //========================================
    gp0.title("Surf");
    gp0.surf(A_mat);
    cin.ignore();  

    gp0.send2gp("reset");
    //========================================
    gp0.title("Image");
    gp0.image(A_mat);
    cin.ignore();  
    
    //========================================
    gp0.title("Image - greyscale");
    gp0.send2gp("set palette grey");
    gp0.image(A_mat);
    cin.ignore();  
        
    //========================================
    gp0.title("Image - RGB");
    C_cube*=250;
    gp0.image(C_cube);
    cin.ignore();  

    gp0.close_window();
    
    return 1;
}

