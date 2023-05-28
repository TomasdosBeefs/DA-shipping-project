//
// Created by ritac on 24/05/2023.
//

#include "Menu.h"
#include <string>
#include <iostream>
#include "Graph.h"
#include "VertexEdge.h"

void Menu::MainMenu(){

    std::cout << "\n\n ----------------------------------------------\n"
                 "|                 Main Menu                    |\n"
                 " ----------------------------------------------\n";
    std::cout << "Hello, travelling salesman!\n";
    std::cout << "Select the number of the topic...\n"
                 "[1]> Backtracking algorithm (4.1)\n"
                 "[2]> Triangular Approximation Heuristic (4.2)\n"
                 "[3]> Other Heuristics (4.3)\n"

                 "\n[0]> Quit\n";

    int topic_in_main_menu;
    std::string striTemp;

    while (true) {
        topic_in_main_menu = 0;
        striTemp = "";
        std::cin >> striTemp;
        try {
            topic_in_main_menu = stoi(striTemp);
        }
        catch (...) {
            topic_in_main_menu = 100;
        }


        if (topic_in_main_menu == 1) {

            //backToMainMenu();
            break;
        } else if (topic_in_main_menu == 2) {


            //backToMainMenu();
            break;
        } else if (topic_in_main_menu == 3) {

            //backToMainMenu();
            break;
        }
        else if (topic_in_main_menu == 0) break;
        else std::cout << "Error: Choose one number of the Main Menu.\n";
}
}

void Menu::GraphMenu() {

    std::cout << "\n\n ----------------------------------------------\n"
                 "|                 Graph Menu                   |\n"
                 " ----------------------------------------------\n";
    std::cout << "Hello, travelling salesman!\n";
    std::cout << "Select the number of the topic...\n"
                 "[1]> Shipping Toy-Graph\n"
                 "[2]> Stadiums Toy-Graph\n"
                 "[3]> Tourism Toy-Graph\n"
                 "[4]> Graph 1 Real-world Graph\n"
                 "[5]> Graph 2 Real-world Graph\n"
                 "[6]> Graph 3 Real-world Graph\n"
                 "[7]> Extra Fully Connected Graph\n"

                 "\n[0]> Quit\n";

    int topic_in_graph_menu;
    std::string striTemp;

    while (true) {
        topic_in_graph_menu = 0;
        striTemp = "";
        std::cin >> striTemp;
        try {
            topic_in_graph_menu = stoi(striTemp);
        }
        catch (...) {
            topic_in_graph_menu = 100;
        }


        if (topic_in_graph_menu == 1) {
            file.readEdges("Toy-Graphs/shipping.csv");
            break;
        } else if (topic_in_graph_menu == 2) {
            file.readEdges("Toy-Graphs/stadiums.csv");
            break;
        } else if (topic_in_graph_menu == 3) {
            file.readTourism();
            break;
        } else if (topic_in_graph_menu == 4) {
            file.readEdges("Real-world Graphs/graph1/edges.csv");
            file.readRealNodes("graph1/nodes.csv");
            break;
        } else if (topic_in_graph_menu == 5) {
            file.readEdges("Real-world Graphs/graph2/edges.csv");
            file.readRealNodes("graph2/nodes.csv");
            break;
        } else if (topic_in_graph_menu == 6) {
            file.readEdges("Real-world Graphs/graph3/edges.csv");
            file.readRealNodes("graph3/nodes.csv");
            break;
        } else if (topic_in_graph_menu == 7) {
            std::cout << "Select the fully connected graph you want to use (ex: edges_25.csv) ... " <<std::endl;
            std::string edgesFile;
            std::cin >> edgesFile;

            file.readEdges("Extra_Fully_Connected_Graphs/" + edgesFile);
            break;
        }

        else if (topic_in_graph_menu == 0) break;
        else std::cout << "Error: Choose one number of the Main Menu.\n";
    }
}

Menu::Menu() {

}



/*void Menu::Error(const std::string &erro) {
    std::cout << "Error: " << erro << std::endl;

}*/

/*void Menu::backToMainMenu( ){
    std::cout << "\n[1]> Back to Main Menu.\n"
                 "[0]> Quit.\n";
    int back;
    std::string striBack;
    while (true) {
        back = 0;
        striBack = "";
        std::cin >> striBack;
        try {
            back = stoi(striBack);
        }
        catch (...) {
            back = 10;
        }
        if (back == 1) Menu menu();
        else if (back == 0) break;
        std::cout << "> Invalid choice.\n"
                     "[1]> Back to Main Menu.\n"
                     "[0]> Quit.\n";
    }

}*/