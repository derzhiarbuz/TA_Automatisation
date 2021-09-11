#ifndef _WIREFMX3DOBJECTGENERATOR
#define _WIREFMX3DOBJECTGENERATOR

#include <fmx.h>
#ifdef _WIN32
#include <tchar.h>
#endif

class WireFMX3DObjectGenerator {
	TPoint3D lastPoint;
public:
	WireFMX3DObjectGenerator() {
		lastPoint.X = 0;
		lastPoint.Y = 0;
		lastPoint.Z = 0;
	}
	TDummy* getWireForLine(TPoint3D *from, TPoint3D *to, TComponent *aOwner, TMaterialSource *ms = NULL);
	TDummy* getObjectForGCodeString(UnicodeString str, TComponent *aOwner, TMaterialSource *ms = NULL);
};

#endif
