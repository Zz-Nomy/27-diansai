
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)

static struct { uint32_t IDR; } port;
#define GPIOB (&port)
#define GPIO_Pin_6 64U
#define GPIO_Pin_7 128U
#define SCL_read      (GPIOB->IDR  & GPIO_Pin_6) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      (GPIOB->IDR  & GPIO_Pin_7) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */
int main(int argc,char **argv) {
    port.IDR=1; assert(!SDA_read && !SCL_read);
    port.IDR=129; assert(SDA_read && !SCL_read);
    port.IDR=65; assert(SCL_read && !SDA_read);
    puts("passed");return 0;
}
