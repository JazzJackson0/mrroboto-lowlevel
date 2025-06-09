#include <stdio.h>
#include "../include/tasks.h"


int main(int argc, char *argv) {

    stdio_init_all();
    // if (cyw43_arch_init()) {
    //     printf("Wi-Fi init failed");
    //     return -1;
    // }

    startTasks();

    while (1) {
        tight_loop_contents();
    }

    // cyw43_arch_deinit();
}







