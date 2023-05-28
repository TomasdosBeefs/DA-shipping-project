//
// Created by ritac on 24/05/2023.
//

#ifndef DA_SHIPPING_PROJECT_MENU_H
#define DA_SHIPPING_PROJECT_MENU_H

#include <iostream>
#include "File_Reader.h"


class Menu {
public:
    Menu();
    void MainMenu();
    void GraphMenu();

    File_Reader file;

//void backToMainMenu( );
    //void Error(const std::string &erro);
};


#endif //DA_SHIPPING_PROJECT_MENU_H