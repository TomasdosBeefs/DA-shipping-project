
#include <iostream>
#include "Menu.h"
#include "File_Reader.h"
#include "Graph.h"


int main() {



    Menu menu;

    try {
        menu.GraphMenu();
        menu.MainMenu();
    }
    catch (std::ios_base::failure &fail) {
        //Menu::Error(fail.what());
        return 1;
    }
    catch (std::exception &ex) {
        //Menu::Error(ex.what());
        return 1;
    }

    return 0;
}