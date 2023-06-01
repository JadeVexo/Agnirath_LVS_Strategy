// #include <iostream>
// #include <fstream>
// #include <unistd.h>
// #include <string>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>
// #include <sys/socket.h>

// int main() {
//     int serverSocket, clientSocket;
//     struct sockaddr_in serverAddress, clientAddress;

//     // Create server socket
//     serverSocket = socket(AF_INET, SOCK_STREAM, 0);
//     if (serverSocket == -1) {
//         std::cerr << "Failed to create server socket." << std::endl;
//         return -1;
//     }

//     serverAddress.sin_family = AF_INET;
//     serverAddress.sin_port = htons(8888);
//     serverAddress.sin_addr.s_addr = INADDR_ANY;

//     // Bind the server socket to a port
//     if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) {
//         std::cerr << "Failed to bind server socket." << std::endl;
//         return -1;
//     }

//     // Listen for incoming connections
//     if (listen(serverSocket, 1) == -1) {
//         std::cerr << "Failed to listen for incoming connections." << std::endl;
//         return -1;
//     }

//     socklen_t clientAddressSize = sizeof(clientAddress);
//     // Accept client connection
//     clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientAddressSize);
//     if (clientSocket == -1) {
//         std::cerr << "Failed to accept client connection." << std::endl;
//         return -1;
//     }

//     close(serverSocket);  // Close the server socket

//     // Send speed data to the client
//     for (int speed = 0; speed <= 150; ++speed) {
//         std::string speed_str = std::to_string(speed) + "\n";
//         send(clientSocket, speed_str.c_str(), speed_str.length(), 0);
//         sleep(1);
//     }

//     for (int speed = 150; speed >= 0; --speed) {
//         std::string speed_str = std::to_string(speed) + "\n";
//         send(clientSocket, speed_str.c_str(), speed_str.length(), 0);
//         sleep(1);
//     }


//     close(clientSocket);  // Close the client socket

//     return 0;
// }

#include <iostream>
#include <unistd.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstdlib>
#include <ctime>

int main() {
    int serverSocket, clientSocket;
    struct sockaddr_in serverAddress, clientAddress;

    // Create server socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Failed to create server socket." << std::endl;
        return -1;
    }

    // Allow reuse of local addresses
    int reuse = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
        std::cerr << "Failed to set socket options." << std::endl;
        return -1;
    }

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8888);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    // Bind the server socket to a port
    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Failed to bind server socket." << std::endl;
        return -1;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 1) == -1) {
        std::cerr << "Failed to listen for incoming connections." << std::endl;
        return -1;
    }

    socklen_t clientAddressSize = sizeof(clientAddress);
    // Accept client connection
    clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientAddressSize);
    if (clientSocket == -1) {
        std::cerr << "Failed to accept client connection." << std::endl;
        return -1;
    }

    close(serverSocket);  // Close the server socket

    // Send variable data to the client
    int speed = 0;
    int rpm = 0;
    int regen = 0;
    int battery = 0;
    int disrem = 3000;
    int mode = 0;
    bool brake = false;
    bool horn = false;
    bool radio = false;
    bool cruise = false;
    bool Lindicator = false;
    bool Rindicator = false;

    std::srand(std::time(0));  // Seed for random number generation

    while (true) {
        // Update the variables
        speed += 1;
        if (speed > 150)
            speed = 0;

        mode += 1;
        if (mode > 3)
            mode = 0;

        rpm += 10;
        if (rpm > 1700)
            rpm = 0;

        regen += 1;
        if (regen > 100)
            regen = 0;

        battery += 1;
        if (battery > 100)
            battery = 0;

        disrem -= 10;
        if (disrem < 0)
            disrem = 3000;

        brake = std::rand() % 2;
        horn = std::rand() % 2;
        radio = std::rand() % 2;
        cruise = std::rand() % 2;
        Lindicator = std::rand() % 2;
        Rindicator = std::rand() % 2;

        // Prepare the variable data string
        std::string variable_data = std::to_string(speed) + " " + std::to_string(rpm) + " " + std::to_string(mode) + " " + std::to_string(regen)
                                    + " " + std::to_string(battery) + " " + std::to_string(disrem) + " " +  std::to_string(brake) + " " 
                                    + std::to_string(horn) + " " + std::to_string(radio) + " "
                                    + std::to_string(cruise) + " " + std::to_string(Lindicator) + " "
                                    + std::to_string(Rindicator) + "\n";

        // Send the variable data to the client
        send(clientSocket, variable_data.c_str(), variable_data.length(), 0);
        usleep(1000000);  // Sleep for 1 second
    }

    close(clientSocket);  // Close the client socket

    return 0;
}
