#include "WireFMX3DObjectGenerator.h"
#include <sstream>

TDummy* WireFMX3DObjectGenerator::getWireForLine(TPoint3D *from, TPoint3D *to, TComponent *aOwner, TMaterialSource *ms) {
	TDummy *newDummy = new TDummy(aOwner);
	TCube *newWire = new TCube(aOwner);
	if(ms)
		newWire->MaterialSource = ms;
	newDummy->AddObject(newWire);

	newDummy->Position->X = from->X;
	newDummy->Position->Y = from->Y;
	newDummy->Position->Z = from->Z;

	TPoint3D *v = new TPoint3D;
	v->X = to->X-from->X;
	v->Y = to->Y-from->Y;
	v->Z = to->Z-from->Z;

	double vLen = sqrt(v->X*v->X + v->Y*v->Y + v->Z*v->Z);
	double yAngle, zAngle;

	if((vLen-fabs(v->Z))<0.0001) { //vertical
		newWire->Scale->X = 1;
		newWire->Scale->Y = 1;
		newWire->Scale->Z = vLen;
		newWire->Position->Z = vLen/2;
	}
	else { //not vertical
        newWire->Scale->X = vLen;
		newWire->Scale->Y = 20;
		newWire->Scale->Z = 5;
		newWire->Position->X = vLen/2;

		zAngle = asin(v->Y/vLen);
		if(v->X<0) zAngle = M_PI - zAngle;

		newDummy->RotationAngle->Z = zAngle/M_PI*180;

		yAngle = -asin(v->Z/vLen);

		newDummy->RotationAngle->Y = yAngle/M_PI*180;
	}

	delete [] v;

    return newDummy;
}


TDummy* WireFMX3DObjectGenerator::getObjectForGCodeString(UnicodeString str, TComponent *aOwner, TMaterialSource *ms) {
    int l = str.Length();
	char *cStr = new char[l+1];
	for(int i=0; i<l; i++)
		cStr[i] = str[i+1];
	cStr[l] = 0;

	std::stringstream strm;
	strm.str(cStr);

	TPoint3D p;
	p = lastPoint;
	int cmd;
	char c=' ';

	while(c==' ' && !strm.eof()) strm>>c;

	if(c!='g' && c!='G') {delete [] cStr; return NULL;}
	strm>>cmd;
	if(cmd!=1) {delete [] cStr; return NULL;}

	while(1) {
        c=' ';
		while(c==' ' && !strm.eof()) strm>>c;
		if(c=='x' || c=='X')
			strm>>p.X;
		else
		if(c=='y' || c=='Y')
			strm>>p.Y;
		else
		if(c=='z' || c=='Z')
			strm>>p.Z;
        else break;
	}

	delete [] cStr;

	TDummy *dum = this->getWireForLine(&lastPoint, &p, aOwner, ms);
	this->lastPoint = p;
	return dum;
}


