#include <math.h>
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <fstream>

#define M_PI		3.14159265358979323846

using namespace std;

void makeHexBarrelVase();
void makeHexBarrelSwirlVase();
void makeOverhangTest();
void makeOverhangTest4mm();
void makeSquareSwirlVase();
void makeDome();
void makeDomeWithEdges();

int main()
{
	//makeHexBarrelVase();
	//makeHexBarrelSwirlVase();
	//makeSquareSwirlVase();
	//makeOverhangTest();
	//makeOverhangTest3mm();
	makeDome();
	getch();
	return 0;
}

void makeHexBarrelVase() {
	ofstream f("vase_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);
	float r, theta, w, l, R, alpha, h2;
	w = 62;
	h2 = 90;
	alpha = M_PI*(2.0/18.0);
	R = h2/sin(alpha);
	l = R*cos(alpha)-w;

	for(float z=0; z<210; z+=5) {
		r=sqrt(R*R-(z-h2)*(z-h2))-l;
		theta = .0;
		f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<6; i++) {
			theta += M_PI/3.0;
			f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<";\n";
		}
	}
}


void makeHexBarrelSwirlVase() {
	ofstream f("swirl_vase_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);
	float r, theta, w, l, R, alpha, h2;
	w = 62;
	h2 = 90;
	alpha = M_PI*(4.0/18.0);
	R = h2/sin(alpha);
	l = R*cos(alpha)-w;

	for(float z=0; z<210; z+=5) {
		r=sqrt(R*R-(z-h2)*(z-h2))-l;
		theta = z*0.015;
		f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<6; i++) {
			theta += M_PI/3.0;
			f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<";\n";
		}
	}
}

void makeSquareSwirlVase() {
	ofstream f("square_vase_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);
	float r, theta, w, l, R, alpha, h2;
	w = 62;
	h2 = 90;

	for(float z=0; z<210; z+=5) {
		r=90;
		theta = z*0.008;
		f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<4; i++) {
			theta += M_PI/2.0;
			f<<"G01X"<<r*cos(theta)<<"Y"<<r*sin(theta)<<";\n";
		}
	}
}


void makeOverhangTest() {
	ofstream f("overhang_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);

	float x=150.0;
	float z=0;
	float fi = M_PI*(30.0/180.0);
	for(int i=0; i<4; i++) {
		for(int j=0; j<5; j++) {
			z+=5.;
			f<<"G01X"<<x<<"Y"<<0<<";\n";
			f<<"G01X"<<x<<"Y"<<100<<";\n";
			f<<"G01X"<<0<<"Y"<<100<<";\n";
            f<<"G01X"<<0<<"Y"<<0<<";\n";
			f<<"G01Z"<<z<<";\n";
			x-=6*tan(fi);
		}
        fi += M_PI*(5.0/180.0);
	}

    f<<"G01X"<<-100<<"Y"<<0<<";\n";
}

void makeOverhangTest4mm() {
	ofstream f("overhang_test3mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);

	float x=150.0;
	float z=0;
	float fi = M_PI*(30.0/180.0);
	for(int i=0; i<5; i++) {
		for(int j=0; j<5; j++) {
			z+=4.;
			f<<"G01X"<<x<<"Y"<<0<<";\n";
			f<<"G01X"<<x<<"Y"<<100<<";\n";
			f<<"G01X"<<0<<"Y"<<100<<";\n";
            f<<"G01X"<<0<<"Y"<<0<<";\n";
			f<<"G01Z"<<z<<";\n";
			x-=6*tan(fi);
		}
        fi += M_PI*(5.0/180.0);
	}

    f<<"G01X"<<-100<<"Y"<<0<<";\n";
}

void makeDome() {
	ofstream f("octodome_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);
	float r, r2, R0, R, theta, alpha1, alpha2, h, h_sph;
	int n;
	n = 4; //количество граней купола
	R0 = 150; //радиус основания купола (вписанная окружность)
	alpha1 = -M_PI/12;// M_PI/6; //угол расширения купола
    alpha2 = M_PI*(60./180); //угол сужения купола
	h = R0*tan(alpha1); //высота центра сферической части купола
	R = R0/cos(alpha1); //радиус сферы купола (вписаной)
	h_sph = h + R*sin(alpha2); //высота перехода от сферы к конусу
	float z;

	//построение сферы
	for(z=0; z<=h_sph; z+=5) {
		r=sqrt(R*R-(h-z)*(h-z)); //радиус вписаной в многоугольник окружности
		r2 = r/cos(M_PI*(1./n)); //радиус описаной окружности
		theta = -M_PI*(1./n);
		f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<n-1; i++) {
			theta += M_PI*(2./n);
			f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<";\n";
		}
		cout<<z<<" "<<asin((z-h)/R)/M_PI*180.<<endl;
	}
	//построение конуса
	r-=5*tan(alpha2);
	while(r>4) {
		r2 = r/cos(M_PI*(1./n)); //радиус описаной окружности
		theta = -M_PI*(1./n);
		f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<n-1; i++) {
			theta += M_PI*(2./n);
			f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<";\n";
		}
		cout<<z<<" "<<alpha2/M_PI*180.<<endl;
		z+=5;
		r-=5*tan(alpha2);
	}
	f<<"G01X-100Y-100;\n";
    f.close();
}

void makeDomeWithEdges() {
    ofstream f("dome_edged_test5mm.gcode");
	f.precision(2);
	f.unsetf(ios::scientific);
	f.setf(ios::fixed);
	float r, r2, R0, R, W, L, theta, alpha1, alpha2, h, h_sph;
	int n;
	n = 4; //количество граней купола
	R0 = 150; //радиус основания купола (вписанная окружность)
	alpha1 = -M_PI/12;// M_PI/6; //угол расширения купола
    alpha2 = M_PI*(60./180); //угол сужения купола
	h = R0*tan(alpha1); //высота центра сферической части купола
	R = R0/cos(alpha1); //радиус сферы купола (вписаной)
	h_sph = h + R*sin(alpha2); //высота перехода от сферы к конусу
	W = 25; //толщина ребра жёсткости
	L = 80; //высота ребра жёсткости
	float z;

	//построение сферы
	for(z=0; z<=h_sph; z+=5) {
		r=sqrt(R*R-(h-z)*(h-z)); //радиус вписаной в многоугольник окружности
		r2 = r/cos(M_PI*(1./n)); //радиус описаной окружности
		theta = -M_PI*(1./n);
		f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<n-1; i++) {
			theta += M_PI*(2./n);
			f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<";\n";
		}
		cout<<z<<" "<<asin((z-h)/R)/M_PI*180.<<endl;
	}
	//построение конуса
	r-=5*tan(alpha2);
	while(r>4) {
		r2 = r/cos(M_PI*(1./n)); //радиус описаной окружности
		theta = -M_PI*(1./n);
		f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<"Z"<<z<<";\n";
		for(int i=0; i<n-1; i++) {
			theta += M_PI*(2./n);
			f<<"G01X"<<r2*cos(theta)<<"Y"<<r2*sin(theta)<<";\n";
		}
		cout<<z<<" "<<alpha2/M_PI*180.<<endl;
		z+=5;
		r-=5*tan(alpha2);
	}
	f<<"G01X-100Y-100;\n";
    f.close();
}
