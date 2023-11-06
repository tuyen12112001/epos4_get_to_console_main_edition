#include "Source.h"

int main()
{
    DeviceController get_to_console;
    get_to_console.configureDevice();

    while (true) {
        std::cout << "Select an option:\n";
        std::cout << "1. Select operation mode\n";
        std::cout << "2. Close device\n";
        std::cout << "Enter your choice (1 or 2): ";
        int choice;
        std::cin >> choice;

        switch (choice) {
        case 1:
            get_to_console.selectOperationMode();
            break;
        case 2:
            get_to_console.closeDevice();
            return 0;
        default:
            std::cerr << "Invalid choice, please try again.\n";
            break;
        }
    }

    return 0;
}
