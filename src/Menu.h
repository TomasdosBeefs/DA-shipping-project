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
    /**

    @brief Displays the main menu.
    The function displays the main menu options to the user, where they can pick the algorithm to run.
    */
    void MainMenu();

    /**

    @brief Displays the graph menu.
    The function displays the graph options to the user, where they can pick the graph/file used to run test the algorithms.
    */
    void GraphMenu();

    File_Reader file;

    /**
     * @brief Measures the execution time of a provided function.
     *
     * This function calculates the execution time of the provided function by recording the start and end timestamps
     * using the high-resolution clock. It returns the execution time in microseconds.
     *
     * @tparam Func The type of the function to be executed.
     * @param func The function to be executed.
     * @return The execution time in microseconds.
     */
    template<typename Func>
    double exec_time(Func&& func){
        auto startTime = std::chrono::high_resolution_clock::now();

        func();

        auto endTime = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

        return duration.count();
    }

    /**
     * @brief Returns to the main menu.
     *
     * This function is responsible for returning to the main menu after completing a specific operation or task.
     * It provides a way for the user to navigate back to the main menu and choose different options.
     */
    void backToMainMenu();
};


#endif //DA_SHIPPING_PROJECT_MENU_H