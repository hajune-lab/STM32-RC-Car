/* Core/Src/main.c -- initialization and the cooperative main loop only. */
#include "main.h"
#include "app.h"

int main(void)
{
    App_Init();

    while (1)
    {
        App_Task();
    }
}
