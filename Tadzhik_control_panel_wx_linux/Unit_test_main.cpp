#include <iostream>
//#include <conio.h>
#include "CNCManager.h"
#include "TadzhikController.h"

using namespace std;

int main()
{
    int16_t data[4] = {1,2,3,4};
    CNCManager manager;
 /*   manager.connect(15, 115200);
    if(!manager.conected()) cout<<"Unable to connect"<<endl;

    for(int i=0; i<100; i++) manager.checkSerial();
    manager.sendData((char*)data, 8);
    getch();
*/
    TadzhikController ctrl;
    ctrl.convertGCodeToPath("../../two_towers.gcode");
 //   ctrl.smoothPath(5.);

    cout<<"\nHello mf";
    return 0;
}
